"""Bounded real headless physics/RL check of the first three path segments."""
import csv
import bisect
from datetime import datetime
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
import numpy as np

import rclpy
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[2]
ACCEPTANCE=PROJECT/"src/go2_mujoco_lidar/test/acceptance"
sys.path.insert(0,str(ACCEPTANCE))
from owned_processes import OwnedProcesses
from path_tracker import PathTracker

output=ROOT/(("distance-info-" if "--distance-info" in sys.argv else "headless-first3walls-")+datetime.now().strftime("%H%M%S"))
output.mkdir()
rclpy.init()
node=rclpy.create_node("g02_headless_driver_check")
def pump():
    rclpy.spin_once(node,timeout_sec=.02)
manager=OwnedProcesses(pump)
logs=[]
fifo=output/"pad.fifo"
os.mkfifo(fifo)
writer=os.open(fifo,os.O_RDWR|os.O_NONBLOCK)
def event(kind,number,value):
    os.write(writer,struct.pack("<IhBB",int(time.monotonic()*1000)&0xffffffff,
                               round(value),kind,number))
def truth():
    path=output/"truth.csv"
    if not path.exists():
        return None
    rows=list(csv.DictReader(path.read_text().splitlines()))
    if not rows:
        return None
    p={k:float(v) for k,v in rows[-1].items()}
    p["yaw"]=math.atan2(2*(p["qw"]*p["qz"]+p["qx"]*p["qy"]),
                        1-2*(p["qy"]**2+p["qz"]**2))
    return p
def until(predicate,seconds):
    end=time.monotonic()+seconds
    while not predicate():
        if time.monotonic()>end:
            raise TimeoutError("headless readiness/progress deadline")
        if any(p.poll() is not None for p in manager.processes):
            raise RuntimeError("owned headless process ended")
        pump()
def advance(seconds):
    end=truth()["sim_time"]+seconds
    until(lambda:truth()["sim_time"]>=end,seconds+5)
def start(name,argv,env,cwd=None):
    f=(output/(name+".log")).open("w")
    logs.append(f)
    return manager.start(argv,env=env,cwd=cwd,stdout=f,stderr=f)

result={"passed":False}
rows=[]
try:
    deps=Path.home()/".local/share/go2-local-navigation-sim"
    native=os.environ.copy()
    native.pop("LD_LIBRARY_PATH",None)
    native.update(LD_PRELOAD=str(ROOT/"physics_observer.so"),
                  G02_PHYSICS_LOG=str(output/"physics"))
    world=PROJECT.parent/"go2_nav2_mujoco_bridge/worlds/go2_complex_indoor.xml"
    sim=start("simulator",[str(deps/"acceptance-build/acceptance_simulator"),
        "-s",str(world),"-i","41","-n","lo","--dds-config",str(ACCEPTANCE/"cyclonedds.xml"),
        "--fifo",str(fifo),"--output",str(output)],native)
    native.pop("LD_PRELOAD")
    ctrl_dir=deps/"acceptance-build/rlroot/deploy/robots/go2/build"
    ctrl=start("controller",[str(ctrl_dir/"go2_ctrl"),"-n","lo","-d","41"],native,ctrl_dir)
    until(lambda:truth() is not None and "FSM: Start Passive" in
          (output/"controller.log").read_text(),25)
    if "--distance-info" in sys.argv:
        print((output/"simulator.log").read_text(),flush=True)
        print((output/"physics-geoms.csv").read_text(),flush=True)
        raise SystemExit(0)
    event(2,5,32767);advance(.25)
    event(1,0,1);advance(.3)
    event(1,0,0);event(2,5,0)
    until(lambda:"to FixStand" in (output/"controller.log").read_text(),10)
    advance(3)
    event(1,11,1);event(2,7,32767);advance(.3)
    event(1,11,0);event(2,7,0)
    until(lambda:"to Velocity_Down" in (output/"controller.log").read_text(),10)
    full_route=json.loads((ROOT/"route.json").read_text())["waypoints"]
    faces=[]
    last_wall=0
    for index,point in enumerate(full_route):
        if point["kind"]=="wall":
            face=(point["wall"],point.get("side"))
            if not faces or face!=faces[-1]:
                faces.append(face)
            if len(faces)>3:
                break
            last_wall=index
    route=full_route[:last_wall+1]
    tracker=PathTracker(route)
    contacts=(output/"physics-contacts.csv").open()
    contacts.seek(0,2)
    intervals={}
    contact_count=0
    max_depth=0.
    max_duration=0.
    wall_clear=[]
    last_face=None
    began=time.monotonic()
    progressed=began
    best=0.
    print("HEADLESS_FIRST3_STARTED",flush=True)
    while True:
        p=truth()
        command,info=tracker.command(p)
        point=tracker.waypoints[info["index"]]
        rows.append(dict(info,truth=p,command=command,kind=point["kind"]))
        if point["kind"]=="wall":
            face=(point["wall"],point.get("side"))
            if face!=last_face:
                line=f"{datetime.now():%H:%M:%S} | G02/headless-wall | 벽면 {face}, arc {tracker.s:.2f}m 추종 중 | -"
                with (ROOT/"live/progress.log").open("a") as stream:
                    stream.write(line+"\n")
                print(line,flush=True)
                last_face=face
        for fields in csv.reader(contacts.readlines()):
            stamp,depth=float(fields[0]),-float(fields[3])
            key=(fields[1],fields[4])
            first,last=intervals.get(key,(stamp,stamp))
            if stamp-last>.003:
                first=stamp
            intervals[key]=(first,stamp)
            contact_count+=1
            max_depth=max(max_depth,depth)
            max_duration=max(max_duration,stamp-first)
            if depth>.005 or stamp-first>1:
                raise RuntimeError("headless collision threshold "+str(fields))
        if info["done"]:
            break
        if info["s_m"]>best+.04:
            best=info["s_m"];progressed=time.monotonic()
        if time.monotonic()-progressed>25 or time.monotonic()-began>180:
            raise RuntimeError("headless path progress stalled "+str(rows[-1]))
        vx,vy,wz=command
        event(2,1,-vx/(4 if vx>=0 else 2)*32767)
        event(2,0,-vy*32767)
        event(2,2,-wz/2*32767)
        previous=p["sim_time"]
        until(lambda:truth()["sim_time"]>previous,.5)
    for axis in (0,1,2):
        event(2,axis,0)
    cross=max(abs(r["cross_track_m"]) for r in rows)
    result.update(passed=cross<=.10,max_cross_track_m=cross,
                  progressed_m=tracker.s,length_m=float(tracker.arc[-1]),
                  wall_sec=time.monotonic()-began,
                  nonfoot_contact_samples=contact_count,max_contact_depth_m=max_depth,
                  max_contact_duration_s=max_duration)
finally:
    for p in reversed(manager.processes):
        manager.stop(p)
    result["residue"]=manager.remaining()
    result["trace"]=rows
    if rows:
        stamps=[r["truth"]["sim_time"] for r in rows]
        for record in csv.DictReader((output/"physics-truth.csv").open()):
            stamp=float(record["time"])
            index=bisect.bisect_right(stamps,stamp)-1
            value=float(record["clearance"])
            if (index>=0 and stamp<=stamps[-1] and rows[index]["kind"]=="wall"
                    and math.isfinite(value)):
                wall_clear.append(value)
        if wall_clear:
            result["wall_clearance_m"]={"median":float(np.median(wall_clear)),
                "p10":float(np.percentile(wall_clear,10)),"min":min(wall_clear)}
    result["passed"]=result["passed"] and not result["residue"]
    (output/"result.json").write_text(json.dumps(result,indent=2))
    os.close(writer)
    fifo.unlink()
    node.destroy_node();rclpy.shutdown()
print("HEADLESS_FIRST3_RESULT "+json.dumps({k:v for k,v in result.items() if k!="trace"}),flush=True)
sys.exit(0 if result["passed"] else 1)
