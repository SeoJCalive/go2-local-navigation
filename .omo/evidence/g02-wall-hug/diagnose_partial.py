"""Offline partial-map diagnosis. Read original physics/SLAM logs; never drive.

Karto BEST POSE uses the LaserScan sensor frame. The installed converter targets
base, so its Karto sensor offset is identity, despite the physical lidar TF.
Original target node IDs were not instrumented; inferred nearest nodes are
explicitly not graph edges.
"""
import ast
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
import shutil
import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from geometry import distance,obstacles

ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[2]
OUT=ROOT/"diagnosis-5"
OUT.mkdir(exist_ok=True)
inputs=["result.json","physics-truth.csv","physics-contacts.csv","physics-geoms.csv",
        "slam-pose.csv","driver-trace.jsonl","g02-mapping.log","route.json",
        "partial-5-metrics.json","gui_run.py","path_tracker.py","physics_observer.cpp",
        "clearance_2d.h","geometry.py","diagnose_partial.py"]
manifest={}
for name in inputs:
    source=ROOT/name
    shutil.copy2(source,OUT/name)
    digest=hashlib.sha256()
    with source.open("rb") as stream:
        for block in iter(lambda:stream.read(1024*1024),b""):
            digest.update(block)
    manifest[name]={"bytes":source.stat().st_size,"sha256":digest.hexdigest()}
(OUT/"inputs.json").write_text(json.dumps(manifest,indent=2))
r=json.loads((OUT/"result.json").read_text())
trailing_fragments={}
def completed_records(name):
    raw=(OUT/name).read_text()
    boundary=raw.rfind("\n")+1
    trailing_fragments[name]=raw[boundary:]
    return np.genfromtxt(io.StringIO(raw[:boundary]),delimiter=",",names=True)
t=completed_records("physics-truth.csv")
s=completed_records("slam-pose.csv")
yaw,tx,ty=r["saved_map_world_transform"]
sx=np.interp(s["time"],t["time"],t["x"])
sy=np.interp(s["time"],t["time"],t["y"])
expected_x=math.cos(yaw)*sx-math.sin(yaw)*sy+tx
expected_y=math.sin(yaw)*sx+math.cos(yaw)*sy+ty
error=np.hypot(s["x"]-expected_x,s["y"]-expected_y)
unwrapped_yaw=np.unwrap(t["yaw"])
truth_yaw=np.interp(s["time"],t["time"],unwrapped_yaw)+yaw
yaw_error=np.arctan2(np.sin(s["yaw"]-truth_yaw),np.cos(s["yaw"]-truth_yaw))
np.savetxt(OUT/"pose-errors.csv",np.c_[s["time"],sx,sy,s["x"],s["y"],error,yaw_error],
    delimiter=",",header="time,truth_x,truth_y,slam_x,slam_y,xy_error_m,yaw_error_rad",comments="")

tree=ast.parse((OUT/"gui_run.py").read_text())
parser_class=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=="LoopEvents")
namespace={"re":re}
exec(compile(ast.Module(body=[parser_class],type_ignores=[]),"LoopEvents","exec"),namespace)
parser=namespace["LoopEvents"]()
number=r"[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?"
best=None
quality=None
scans=[]
raw_accepted=[]
for line in (OUT/"g02-mapping.log").read_text().splitlines():
    if "BEST POSE =" in line:
        values=re.findall(number,line.split("BEST POSE =",1)[1])
        if len(values)>=6:
            best=list(map(float,values[:3]))
            quality=list(map(float,values[3:6]))
    marker=re.search(r"G02_LOOP event=(\w+) scan=(\d+) time=([\d.]+)",line)
    if marker:
        kind,scan,stamp=marker.groups()
        if kind=="scan_begin":
            scans.append({"scan_id":int(scan),"time":float(stamp),
                "pre_loop_sensor_pose":best,"response_variance":quality})
        if kind=="accepted":
            raw_accepted.append({"scan_id":int(scan),"time":float(stamp)})
    parser.feed(line)

scan_config=PROJECT/"install/go2_inputs/share/go2_inputs/config/mapping_scan.yaml"
shutil.copy2(scan_config,OUT/"installed-mapping-scan.yaml")
parameters=yaml.safe_load(scan_config.read_text())["mapping_scan"]["pointcloud_to_laserscan"]["ros__parameters"]
assert parameters["target_frame"]=="base", "This fixture requires the actual base-frame scan contract."
offset=np.zeros(2)
world=PROJECT.parent/"go2_nav2_mujoco_bridge/worlds/go2_complex_indoor.xml"
geoms=obstacles(world)
def truth_at(stamp):
    x=float(np.interp(stamp,t["time"],t["x"]))
    y=float(np.interp(stamp,t["time"],t["y"]))
    heading=float(np.interp(stamp,t["time"],unwrapped_yaw))
    sensor=np.array([x+math.cos(heading)*offset[0]-math.sin(heading)*offset[1],
                     y+math.sin(heading)*offset[0]+math.cos(heading)*offset[1]])
    mapped=np.array([math.cos(yaw)*sensor[0]-math.sin(yaw)*sensor[1]+tx,
                     math.sin(yaw)*sensor[0]+math.cos(yaw)*sensor[1]+ty])
    return sensor,mapped

for scan in scans:
    sensor,expected=truth_at(scan["time"])
    scan["truth_sensor_map_xy"]=expected.tolist()
    scan["scan_origin_to_world_surface_m"]=float(distance(sensor,geoms))
    estimate=scan["pre_loop_sensor_pose"]
    scan["pre_loop_sensor_xy_error_m"]=(
        float(np.linalg.norm(np.array(estimate[:2])-expected)) if estimate else None)

scan_by_id={row["scan_id"]:row for row in scans}
for event in parser.events:
    stamp=event["time"]
    _,expected=truth_at(stamp)
    candidate=event["candidate_map_pose"]
    event["candidate_frame"]="LaserScan base; identity Karto sensor offset"
    event["truth_sensor_map_xy"]=expected.tolist()
    event["candidate_xy_error_m"]=(
        float(np.linalg.norm(np.array(candidate[:2])-expected)) if candidate else None)
    event["correct_vs_gt_010m"]=(
        (event["candidate_xy_error_m"]<=.10)==event["accepted"] if candidate else None)
    event["target_node_id"]=None
    event["target_node_status"]="not recorded; cannot recover actual graph edge from this log"
    before=np.flatnonzero(s["time"]<stamp)
    after=np.flatnonzero(s["time"]>=stamp)
    event["xy_error_before_m"]=float(error[before[-1]]) if len(before) else None
    event["xy_error_after_m"]=float(error[after[0]]) if len(after) else None
    event["truth_base_world_xy"]=[
        float(np.interp(stamp,t["time"],t["x"])),
        float(np.interp(stamp,t["time"],t["y"]))]
    prior=[row for row in scans if row["scan_id"]<event["scan_id"]-20]
    if candidate and prior:
        nearest=min(prior,key=lambda row:np.linalg.norm(
            np.array(row["truth_sensor_map_xy"])-candidate[:2]))
        event["nearest_gt_prior_node_not_actual_edge"]={
            "scan_id":nearest["scan_id"],"time":nearest["time"],
            "distance_to_candidate_m":float(np.linalg.norm(
                np.array(nearest["truth_sensor_map_xy"])-candidate[:2]))}
accepted=[e for e in parser.events if e["accepted"]]
first={}
for threshold in (.05,.10,.20,.50,1.,2.):
    indices=np.flatnonzero(error>threshold)
    first[str(threshold)]=(
        {"time":float(s["time"][indices[0]]),"xy_error_m":float(error[indices[0]]),
         "truth_xy":[float(sx[indices[0]]),float(sy[indices[0]])]} if len(indices) else None)
maximum=int(np.argmax(error))
summary={"attempt":5,"alignment":r["saved_map_world_transform"],
    "unterminated_csv_fragments_preserved":trailing_fragments,
    "base_pose_error_m":{"max":float(error[maximum]),"p95":float(np.percentile(error,95)),
        "time_of_max":float(s["time"][maximum]),"first_above":first},
    "raw_accepted_count":len(raw_accepted),"parsed_accepted_count":len(accepted),
    "raw_accepted":raw_accepted,"accepted":accepted,
    "all_loop_decisions":len(parser.events),
    "sensor_offset_xy_m":offset.tolist(),
    "scan_converter_parameters":parameters,
    "target_node_provenance":"query IDs recorded; target IDs absent; no invented pairs"}
(OUT/"summary.json").write_text(json.dumps(summary,indent=2))
(OUT/"loop-decisions.json").write_text(json.dumps(parser.events,indent=2))
(OUT/"scan-quality.json").write_text(json.dumps(scans,indent=2))

fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
axes[0].plot(s["time"],error,lw=1,label="SLAM base vs qpos XY")
axes[0].axhline(.10,color="black",ls="--",lw=.8,label="0.10 m")
axes[0].set_ylabel("XY error (m)")
axes[1].plot(s["time"],np.degrees(yaw_error),lw=1)
axes[1].set_ylabel("Yaw error (deg)")
quality_scans=[row for row in scans if row["response_variance"]]
scatter=axes[2].scatter([row["time"] for row in quality_scans],
    [row["response_variance"][0] for row in quality_scans],s=7,
    c=[row["scan_origin_to_world_surface_m"] for row in quality_scans],
    cmap="viridis",vmin=0,vmax=1)
axes[2].set_ylabel("Sequential match response")
axes[2].set_xlabel("Simulator / scan time (s)")
fig.colorbar(scatter,ax=axes[2],label="GT scan-origin-to-surface distance (m)")
for event in accepted:
    color="green" if event["correct_vs_gt_010m"] else "red"
    for axis in axes:
        axis.axvline(event["time"],color=color,alpha=.5,lw=.8)
    axes[0].annotate(str(event["scan_id"]),(event["time"],np.interp(event["time"],s["time"],error)),
                     fontsize=7,rotation=45)
for axis in axes:
    axis.grid(alpha=.25)
axes[0].legend(loc="upper left")
fig.suptitle("Partial 5: base error; accepted loops green=GT-correct, red=incorrect")
fig.tight_layout()
fig.savefig(ROOT/"live/partial-5-error-vs-time.png",dpi=100)
fig.savefig(OUT/"error-vs-time.png",dpi=100)
plt.close(fig)
print(json.dumps({k:v for k,v in summary.items() if k!="accepted"},indent=2))
print("ACCEPTED_DIAGNOSIS")
for event in accepted:
    print(json.dumps(event))
