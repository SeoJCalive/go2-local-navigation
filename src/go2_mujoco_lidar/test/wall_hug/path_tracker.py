"""Maintained G02 lookahead path tracking, qpos to pad commands; promoted from commit 123f8b7 evidence.

Inputs and outputs follow the functions below; see README.md in this directory
and docs/stabilization-2026-11/G02_WALL_HUG.md for provenance and limitations.
Simulation/offline diagnostics only; real robot safety is unconfirmed.
"""
"""Continuous qpos-driven pad tracking; progress is arc length, never point arrival.

Use a short lookahead and cross-track correction while maintaining forward gait
speed. Track the unchanged planned reference. Clearance is an independent test
metric and never changes the pad command. This drives the pad
only and does not change odometry, scans, SLAM input, or physics.
"""
import math
import numpy as np


class PathTracker:
    def __init__(self,waypoints):
        self.waypoints=[]
        for point in waypoints:
            if not self.waypoints or math.dist(point["xy"],self.waypoints[-1]["xy"])>1e-5:
                self.waypoints.append(point)
        self.points=np.array([p["xy"] for p in self.waypoints])
        self.delta=np.diff(self.points,axis=0)
        self.length=np.linalg.norm(self.delta,axis=1)
        self.arc=np.r_[0,np.cumsum(self.length)]
        self.s=0.
        self.index=0

    def command(self,p):
        xy=np.array([p["x"],p["y"]])
        candidates=[]
        for k in range(max(0,self.index-1),min(len(self.length),self.index+9)):
            u=float(np.clip(np.dot(xy-self.points[k],self.delta[k])/self.length[k]**2,0,1))
            arc=self.arc[k]+u*self.length[k]
            if self.s-.1<=arc<=self.s+.65:
                projection=self.points[k]+u*self.delta[k]
                candidates.append((float(np.linalg.norm(xy-projection))+.002*max(0,arc-self.s),
                                   arc,k,projection))
        _,arc,k,projection=min(candidates,key=lambda row:row[0])
        self.s=float(max(self.s,arc))
        self.index=max(self.index,int(np.searchsorted(self.arc,self.s,side="right")-1))
        self.index=min(self.index,len(self.length)-1)
        tangent=self.delta[k]/self.length[k]
        cross=float(tangent[0]*(xy-projection)[1]-tangent[1]*(xy-projection)[0])
        ahead=min(len(self.length)-1,int(np.searchsorted(
            self.arc,self.s+.35,side="right")-1))
        turn=abs(math.atan2(np.cross(tangent,self.delta[ahead]),
                           np.dot(tangent,self.delta[ahead])))>.25
        look=min(self.arc[-1],self.s+(.11 if turn else .22))
        target=np.array([np.interp(look,self.arc,self.points[:,axis]) for axis in (0,1)])
        desired=math.atan2(target[1]-p["y"],target[0]-p["x"])
        error=math.atan2(math.sin(desired-p["yaw"]),math.cos(desired-p["yaw"]))
        vx=.24 if turn else .30
        vy=-math.copysign(min(.16,max(.09,1.4*abs(cross))),cross) if abs(cross)>.025 else 0.
        # Cross-track correction is normal to the path, expressed in body axes.
        normal=np.array([-tangent[1],tangent[0]])
        lateral=vy*normal
        world=vx*np.array([math.cos(p["yaw"]),math.sin(p["yaw"])])+lateral
        wz=max(-.65,min(.65,1.7*error-1.0*cross))
        if abs(error)>.6:
            world*=0
        vx=float(np.dot(world,[math.cos(p["yaw"]),math.sin(p["yaw"])]))
        vy=float(np.dot(world,[-math.sin(p["yaw"]),math.cos(p["yaw"])]))
        return [vx,vy,wz],{"s_m":self.s,"length_m":float(self.arc[-1]),
            "index":self.index,"cross_track_m":cross,"target":target.tolist(),
            "heading_error_rad":error,"corner":turn,
            "done":bool(self.s>=self.arc[-1]-.04)}
