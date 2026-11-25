"""MJCF exposed-face route and independent saved occupancy evaluation.

Only worldbody box/cylinder obstacles enter the oracle. Route center clearance
is a .28 m moving-body planning envelope + .10 m; the .42 m Nav2 circle cannot
fit every architectural gap. Actual geometry clearance/contact is a separate
GUI measurement. Side samples occluded by another obstacle are skipped.
Door/corner transitions use an 8-neighbor grid with diagonal corner cutting
forbidden. Raster and analytic distances independently check each segment.
"""
import heapq
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from PIL import Image, ImageDraw
import yaml

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parents[2]
sys.path.insert(0, str(PROJECT / "src/go2_mujoco_lidar/test/acceptance"))
from route_clearance import obstacle_distance

THRESHOLDS = {
    "wall_offset_p95_m": .05, "wall_offset_max_m": .15,
    "doubled_wall_components": 0, "ghost_wall_components": 0,
    "wall_thickness_p95_m": .15, "slam_pose_max_xy_m": .10,
    "nonfoot_max_depth_m": .015, "nonfoot_sustained_depth_m": .005,
    "nonfoot_max_deep_duration_s": .050, "nonfoot_max_duration_s": 1.,
    "nav2_goal_xy_m": .30,
    "planned_body_radius_m": .28, "planned_open_wall_clearance_m": .10,
}


def obstacles(world):
    return [dict(g.attrib) for g in ET.parse(world).findall("./worldbody/geom")
            if g.get("type") in ("box", "cylinder")]


def distance(points, geoms):
    points = np.asarray(points)
    best = np.full(points.shape[:-1], np.inf)
    for g in geoms:
        x,y,_ = map(float, g["pos"].split())
        size = list(map(float, g["size"].split()))
        d = points - (x,y)
        if g["type"] == "cylinder":
            value = np.linalg.norm(d,axis=-1)-size[0]
        else:
            a = float(g.get("euler","0 0 0").split()[2])
            xx = d[...,0]*math.cos(a)+d[...,1]*math.sin(a)
            yy = -d[...,0]*math.sin(a)+d[...,1]*math.cos(a)
            q = np.stack((np.abs(xx)-size[0], np.abs(yy)-size[1]),axis=-1)
            value = np.linalg.norm(np.maximum(q,0),axis=-1)+np.minimum(np.max(q,axis=-1),0)
        best = np.minimum(best,value)
    return best


def route(world):
    geoms = obstacles(world)
    resolution = .05
    xs, ys = np.arange(-7.9,7.901,resolution),np.arange(-5.9,5.901,resolution)
    xx,yy = np.meshgrid(xs,ys)
    free = distance(np.stack((xx,yy),axis=-1),geoms) >= .36
    def cell(p):
        c=(int(round((p[1]-ys[0])/resolution)),int(round((p[0]-xs[0])/resolution)))
        if free[c]:
            return c
        nearby=[(r,k) for r in range(max(0,c[0]-2),min(len(ys),c[0]+3))
                for k in range(max(0,c[1]-2),min(len(xs),c[1]+3)) if free[r,k]]
        return min(nearby,key=lambda v:math.dist(p,(xs[v[1]],ys[v[0]])))
    def xy(c):
        return [float(xs[c[1]]),float(ys[c[0]])]
    def astar(a,b):
        start,goal=cell(a),cell(b)
        queue=[(0,start)]
        costs={start:0}
        parents={}
        while queue:
            _,u=heapq.heappop(queue)
            if u==goal:
                path=[u]
                while path[-1]!=start:
                    path.append(parents[path[-1]])
                return [xy(c) for c in reversed(path)]
            for dy,dx in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                v=(u[0]+dy,u[1]+dx)
                if not (0<=v[0]<len(ys) and 0<=v[1]<len(xs) and free[v]):
                    continue
                if dx and dy and not(free[u[0],v[1]] and free[v[0],u[1]]):
                    continue
                cost=costs[u]+math.hypot(dx,dy)
                if cost<costs.get(v,math.inf):
                    costs[v]=cost
                    parents[v]=u
                    heapq.heappush(queue,(cost+math.dist(v,goal),v))
        raise RuntimeError(f"no body-clear path: {a} -> {b}")
    segments=[]
    skipped=[]
    for geom in geoms:
        x,y,_=map(float,geom["pos"].split())
        sizes=list(map(float,geom["size"].split()))
        a=float(geom.get("euler","0 0 0").split()[2])
        ca,sa=math.cos(a),math.sin(a)
        def world_point(p):
            return [x+ca*p[0]-sa*p[1],y+sa*p[0]+ca*p[1]]
        if geom["type"]=="box":
            sx,sy=sizes[:2]
            sides=[
                [world_point((q,sy+.38)) for q in np.linspace(-sx,sx,max(2,math.ceil(2*sx/.12)))],
                [world_point((sx+.38,q)) for q in np.linspace(sy,-sy,max(2,math.ceil(2*sy/.12)))],
                [world_point((q,-sy-.38)) for q in np.linspace(sx,-sx,max(2,math.ceil(2*sx/.12)))],
                [world_point((-sx-.38,q)) for q in np.linspace(-sy,sy,max(2,math.ceil(2*sy/.12)))],
            ]
        else:
            sides=[[[x+(sizes[0]+.38)*math.cos(a),y+(sizes[0]+.38)*math.sin(a)]
                    for a in np.linspace(0,2*math.pi,math.ceil(2*math.pi*(sizes[0]+.38)/.12))]]
        for side,points in enumerate(sides):
            chunk=[]
            chunks=[]
            for p in points:
                valid=(-7.9<p[0]<7.9 and -5.9<p[1]<5.9 and distance(p,geoms)>=.365)
                if valid:
                    chunk.append(p)
                elif chunk:
                    chunks.append(chunk);chunk=[]
            if chunk:
                chunks.append(chunk)
            if not chunks:
                skipped.append([geom["name"],side,"occluded/exterior"])
            for c in chunks:
                segments.append({"wall":geom["name"],"side":side,"points":c})
    output=[]
    visited=[]
    current=[0.,0.]
    remaining=segments[:]
    while remaining:
        choices=[(math.dist(current,s["points"][end]),i,end)
                 for i,s in enumerate(remaining) for end in (0,-1)]
        _,i,end=min(choices)
        s=remaining.pop(i)
        points=s["points"] if end==0 else list(reversed(s["points"]))
        transit=astar(current,points[0])
        for p in transit[::3]+[transit[-1]]:
            output.append({"xy":p,"kind":"transit","wall":s["wall"]})
        output.extend({"xy":p,"kind":"wall","wall":s["wall"],"side":s["side"]} for p in points)
        current=points[-1]
        visited.append({"wall":s["wall"],"side":s["side"],"count":len(points)})
    output.extend({"xy":p,"kind":"loop-return","wall":"origin"}
                  for p in astar(current,[0.,0.])[::3])
    output.append({"xy":[0.,0.],"kind":"loop-return","wall":"origin"})
    current=[0.,0.]
    # Preserve explicit centered passes through every bottom-left island gap.
    # They are already in wall-face traversal; record their actual center lines.
    gaps={"south": [[-6.4,-5.285],[-4.0,-5.285]],
          "west": [[-6.875,-5.0],[-6.875,-3.2]],
          "east": [[-3.905,-5.0],[-3.905,-3.2]],
          "north": [[-6.4,-3.145],[-4.0,-3.145]]}
    # Add dedicated centered gap passes and close a second full loop.
    for name,line in gaps.items():
        for p in astar(current,line[0])[::3]:
            output.append({"xy":p,"kind":"transit","wall":"bottom-left-"+name})
        output.append({"xy":line[0],"kind":"gap","wall":name})
        output.append({"xy":line[1],"kind":"gap","wall":name})
        current=line[1]
    output.extend({"xy":p,"kind":"loop-return","wall":"origin"}
                  for p in astar(current,[0.,0.])[::3])
    output.append({"xy":[0.,0.],"kind":"loop-return","wall":"origin"})
    minimum=math.inf
    previous=[0.,0.]
    for row in output:
        p=row["xy"];count=max(1,math.ceil(math.dist(previous,p)/.02))
        d=distance(np.linspace(previous,p,count+1),geoms).min()
        minimum=min(minimum,float(d)-.28)
        previous=p
    if minimum<=0:
        raise RuntimeError(f"route intersects footprint: {minimum}")
    result={"waypoints":output,"faces":visited,"skipped_faces":skipped,
            "minimum_planned_clearance_m":minimum,
            "length_m":sum(math.dist(a["xy"],b["xy"]) for a,b in zip(
                [{"xy":[0,0]}]+output,output)), "bottom_left_gap_centerlines":gaps}
    (ROOT/"route.json").write_text(json.dumps(result,indent=2))
    (ROOT/"thresholds.json").write_text(json.dumps(THRESHOLDS,indent=2))
    rgb=np.full((*free.shape,3),245,dtype=np.uint8)
    rgb[distance(np.stack((xx,yy),axis=-1),geoms)<=0]=[30,30,30]
    canvas=Image.fromarray(rgb[::-1].copy()).resize((len(xs)*3,len(ys)*3))
    draw=ImageDraw.Draw(canvas)
    previous=[0.,0.]
    def plot(p):
        return ((p[0]-xs[0])/resolution*3,(ys[-1]-p[1])/resolution*3)
    for row in output:
        draw.line([plot(previous),plot(row["xy"])],
                  fill=(20,150,70) if row["kind"]=="wall" else (220,110,20),width=1)
        previous=row["xy"]
    canvas.save(ROOT/"live/planned-wall-route.png")
    return result


def evaluate(world, map_yaml, transform, output):
    spec=yaml.safe_load(Path(map_yaml).read_text())
    image=np.array(Image.open(Path(map_yaml).parent/spec["image"]))
    occupied=image<80
    resolution=spec["resolution"]
    ox,oy,_=spec["origin"]
    rr,cc=np.indices(image.shape)
    mx=ox+(cc+.5)*resolution
    my=oy+(image.shape[0]-rr-.5)*resolution
    yaw,tx,ty=transform
    wx=math.cos(yaw)*(mx-tx)+math.sin(yaw)*(my-ty)
    wy=-math.sin(yaw)*(mx-tx)+math.cos(yaw)*(my-ty)
    signed=distance(np.stack((wx,wy),axis=-1),obstacles(world))
    gt=signed<=0
    # Occupied cells inside the physical wall are correct, not offset by the
    # wall half-thickness. Outside cells use exact distance to the solid.
    offset=np.maximum(signed[occupied],0)
    # Ghost/double components must extend >= .20m and lie > .15m from GT.
    ghosts=occupied&(np.abs(signed)>.15)
    labels,n=label(ghosts)
    sizes=np.bincount(labels.ravel())[1:]
    ghost_count=int((sizes>=max(3,round(.20/resolution))).sum())
    thickness=2*distance_transform_edt(occupied)*resolution
    nonzero=thickness[occupied]
    profiles=[]
    doubled=0
    offsets=[]
    widths=[]
    geoms=obstacles(world)
    def pixels(points):
        px=math.cos(yaw)*points[:,0]-math.sin(yaw)*points[:,1]+tx
        py=math.sin(yaw)*points[:,0]+math.cos(yaw)*points[:,1]+ty
        c=np.floor((px-ox)/resolution).astype(int)
        r=image.shape[0]-1-np.floor((py-oy)/resolution).astype(int)
        valid=(r>=0)&(r<image.shape[0])&(c>=0)&(c<image.shape[1])
        values=np.zeros(len(points),dtype=bool)
        values[valid]=occupied[r[valid],c[valid]]
        return values
    for geom in geoms:
        if geom["type"]!="box":
            continue
        x,y,_=map(float,geom["pos"].split())
        sx,sy,_=map(float,geom["size"].split())
        angle=float(geom.get("euler","0 0 0").split()[2])
        ca,sa=math.cos(angle),math.sin(angle)
        # Wall centerline normal profiles exclude end corners by .15m.
        major,minor=(sx,sy) if sx>=sy else (sy,sx)
        if minor>.10:
            continue  # Broad islands are obstacles, not anomalously thick walls.
        direction=np.array([ca,sa]) if sx>=sy else np.array([-sa,ca])
        normal=np.array([-direction[1],direction[0]])
        run=0
        events=0
        measured=0
        for u in np.arange(-major+.15,major-.15,resolution):
            center=np.array([x,y])+u*direction
            others=[g for g in geoms if g["name"]!=geom["name"]]
            if float(distance(center,others))<minor+.35:
                continue  # Intersecting real geometry is not a duplicate wall.
            normal_offsets=np.arange(-minor-.35,minor+.351,resolution/4)
            hits=pixels(center[None,:]+normal_offsets[:,None]*normal)
            # Split occupied intervals; raster nearest-neighbor duplicates collapse.
            indices=np.flatnonzero(hits)
            groups=np.split(indices,np.flatnonzero(np.diff(indices)>1)+1) if len(indices) else []
            groups=[group for group in groups if len(group)>=2]
            near=[group for group in groups if abs(float(np.mean(normal_offsets[group])))<minor+.20]
            if near:
                measured+=1
                for primary in near:
                    center_offset=float(np.mean(normal_offsets[primary]))
                    # A solid partition has two real faces. Do not mislabel its
                    # two observed surfaces as a doubled SLAM wall.
                    width=float(normal_offsets[primary[-1]]-normal_offsets[primary[0]]+resolution/4)
                    spans_wall=(normal_offsets[primary[0]]<0<normal_offsets[primary[-1]]
                                and width>=2*minor-resolution)
                    offsets.append(abs(center_offset) if spans_wall else min(
                        abs(center_offset-minor),abs(center_offset+minor)))
                    widths.append(width)
            sides=[[group for group in near if (
                float(np.mean(normal_offsets[group]))) * sign >= 0 or (
                    normal_offsets[group[0]]<0<normal_offsets[group[-1]])] for sign in (-1,1)]
            has_double=any(len(side)>1 and max(
                float(np.mean(normal_offsets[p])) for p in side)-min(
                float(np.mean(normal_offsets[p])) for p in side)>.10 for side in sides)
            if has_double:
                run+=1
            else:
                if run*resolution>=.20:
                    events+=1
                run=0
        if run*resolution>=.20:
            events+=1
        doubled+=events
        profiles.append({"geom":geom["name"],"normal_profiles":measured,
                         "doubled_runs_ge_020m":events})
    rendered=np.full((*image.shape,3),245,dtype=np.uint8)
    rendered[image==205]=180
    rendered[gt]=[20,90,255]
    rendered[occupied]=[240,55,35]
    rendered[occupied&gt]=[160,35,175]
    Image.fromarray(rendered).save(output)
    metrics={"wall_offset_m":{"median":float(np.median(offset)),"p95":float(np.percentile(offset,95)),
                              "max":float(offset.max())},
             "ghost_wall_components":ghost_count,
             "doubled_wall_components":doubled,
             "normal_profile_offset_m":{
                 "median":float(np.median(offsets)) if offsets else None,
                 "p95":float(np.percentile(offsets,95)) if offsets else None,
                 "max":float(max(offsets)) if offsets else None},
             "normal_profile_thickness_m":{
                 "p95":float(np.percentile(widths,95)) if widths else None,
                 "max":float(max(widths)) if widths else None},
             "wall_profiles":profiles,
             "wall_thickness_m":{"p95":float(np.percentile(nonzero,95)),"max":float(nonzero.max())},
             "occupied_cells":int(occupied.sum()),"resolution_m":resolution,
             "alignment":{"yaw":yaw,"tx":tx,"ty":ty},
             "thresholds":THRESHOLDS}
    return metrics


if __name__=="__main__":
    world=PROJECT.parent/"go2_nav2_mujoco_bridge/worlds/go2_complex_indoor.xml"
    result=route(world)
    print(json.dumps({k:v for k,v in result.items() if k not in ("waypoints","faces")}))
