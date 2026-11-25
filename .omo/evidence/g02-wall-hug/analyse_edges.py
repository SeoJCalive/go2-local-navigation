"""Evaluate actual, saved graph factors against physics and raw scans.

Factor correctness uses the invariant relative pose, not a globally displaced
candidate. Thresholds are 0.10 m and 5 degrees. Raw endpoints are projected in
2D using GT base yaw; wall-hit association allows 0.075 m for projection/scan
quantization. Null scan bins are invalid/empty; their cause is not inferred.
"""
import argparse
import ast
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from geometry import obstacles, distance

ROOT=Path(__file__).resolve().parent
args=argparse.ArgumentParser()
args.add_argument("--attempt",type=int,required=True)
opt=args.parse_args()
run=json.loads((ROOT/"result.json").read_text())
assert run["attempt"]==opt.attempt,"refuse mixed-attempt inputs"
OUT=ROOT/f"analysis-{opt.attempt}"
OUT.mkdir(exist_ok=True)
world=ROOT.parents[2].parent/"go2_nav2_mujoco_bridge/worlds/go2_complex_indoor.xml"
geoms=obstacles(world)
for source,name in (
    (world,"world.xml"),
    (ROOT.parents[2]/"install/go2_nav2/share/go2_nav2/config/slam_mapping.yaml","installed-slam.yaml"),
    (ROOT.parents[2]/"install/go2_inputs/share/go2_inputs/config/mapping_scan.yaml","installed-scan.yaml")):
    shutil.copy2(source,OUT/name)
for extension in ("yaml","pgm"):
    source=ROOT/"maps"/f"wall-{opt.attempt}.{extension}"
    shutil.copy2(source,OUT/source.name)
shutil.copy2(ROOT/"live/final-map-ground-truth-overlay.png",
             ROOT/"live"/f"attempt{opt.attempt}-gt-overlay.png")
manifest={}
for name in ("result.json","map-metrics.json","physics-truth.csv",
             "physics-contacts.csv","physics-geoms.csv","slam-pose.csv",
             "karto-scans.jsonl","karto-edges.jsonl","driver-trace.jsonl",
             f"baseline{opt.attempt}-final-factors.jsonl","route.json","gui_run.py"):
    digest=hashlib.sha256()
    with (ROOT/name).open("rb") as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b""):
            digest.update(chunk)
    manifest[name]={"bytes":(ROOT/name).stat().st_size,"sha256":digest.hexdigest()}
(OUT/"inputs.json").write_text(json.dumps(manifest,indent=2))

uncommitted={}
def committed_lines(name):
    with (ROOT/name).open() as f:
        for line in f:
            if line.endswith("\n"):
                yield line
            else:
                uncommitted[name]=line
def rows(name):
    for row in csv.DictReader(committed_lines(name)):
        if None in row or any(v is None for v in row.values()):
            raise ValueError(f"malformed committed CSV row in {name}: {row}")
        yield row
truth=np.array([[float(r[k]) for k in ("time","x","y","yaw","clearance",
    "closest_world","closest_robot")] for r in rows("physics-truth.csv")])
assert np.all(np.diff(truth[:,0])>=0),"truth clock decreased"
gt_columns=[np.ascontiguousarray(truth[:,k]) for k in (0,1,2,3)]
gt_columns[3]=np.unwrap(gt_columns[3])
def gt(t):
    point=np.array([np.interp(t,gt_columns[0],gt_columns[k]) for k in (1,2,3)])
    point[2]=wrap(point[2])
    return point
def wrap(a):
    return (a+math.pi)%(2*math.pi)-math.pi
def relative(a,b):
    dx,dy=b[:2]-a[:2]
    return np.array([math.cos(a[2])*dx+math.sin(a[2])*dy,
        -math.sin(a[2])*dx+math.cos(a[2])*dy,wrap(b[2]-a[2])])
geomid={int(r["id"]):r for r in rows("physics-geoms.csv")}
valid_clear=truth[np.isfinite(truth[:,4])]
def clearance(t):
    r=valid_clear[np.argmin(abs(valid_clear[:,0]-t))]
    if int(r[5])==-1:
        assert r[4]==.75 and int(r[6])==-1,"invalid observer distance sentinel"
        return {"sample_time":r[0],"clearance_m":None,
            "clearance_lower_bound_m":.75,"world":None,"robot":None,
            "reason":"observer search is capped at 0.75 m"}
    return {"sample_time":r[0],"clearance_m":r[4],
        "world":geomid[int(r[5])]["name"],
        "robot":geomid[int(r[6])]}

with (ROOT/f"baseline{opt.attempt}-final-factors.jsonl").open() as f:
    factors=[json.loads(line) for line in f]
factor_index={}
for f in factors:
    a,b=gt(f["from_time"]),gt(f["to_time"])
    expected=relative(a,b)
    actual=np.array(f["relative_pose"])
    f.update(gt_from_pose=a.tolist(),gt_to_pose=b.tolist(),
        gt_separation_m=float(np.linalg.norm(b[:2]-a[:2])),
        gt_relative_pose=expected.tolist(),
        relative_xy_error_m=float(np.linalg.norm(actual[:2]-expected[:2])),
        relative_yaw_error_deg=math.degrees(abs(wrap(actual[2]-expected[2]))))
    f["correct"]=f["relative_xy_error_m"]<=.10 and f["relative_yaw_error_deg"]<=5.
    factor_index[frozenset((f["from_scan_id"],f["to_scan_id"]))]=f
native_edges=[json.loads(line) for line in committed_lines("karto-edges.jsonl")]
loop_edges=[e for e in native_edges if e["loop"]]

tree=ast.parse((ROOT/"gui_run.py").read_text())
definition=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=="LoopEvents")
namespace={"re":re}
exec(compile(ast.Module(body=[definition],type_ignores=[]),"validated_loop_parser","exec"),namespace)
parser=namespace["LoopEvents"]()
with (ROOT/"g02-mapping.log").open() as log:
    for line in log:
        if "G02_LOOP" in line or "BEST POSE =" in line or "bestPose:" in line:
            parser.feed(line)
accepted=[e for e in parser.events if e["accepted"]]
wanted={i for e in loop_edges for i in (e["from_scan_id"],e["to_scan_id"])}
scans={}
for line in committed_lines("karto-scans.jsonl"):
    scan=json.loads(line)
    if scan["scan_id"] in wanted:
        scans[scan["scan_id"]]=scan
assert wanted<=scans.keys(),"missing actual raw scan"

def scan_geometry(scan):
    a=np.array([np.nan if v is None else v for v in scan["ranges"]])
    valid=np.isfinite(a)&(a>=scan["range_min"])&(a<=scan["range_max"])
    angles=scan["angle_min"]+np.arange(len(a))*scan["angle_increment"]
    pose=gt(scan["time"])
    points=np.column_stack((np.cos(angles[valid]+pose[2])*a[valid]+pose[0],
                            np.sin(angles[valid]+pose[2])*a[valid]+pose[1]))
    residuals=np.array([abs(distance(points,[g])) for g in geoms])
    labels=np.argmin(residuals,axis=0)
    residual=np.min(residuals,axis=0)
    hit=residual<=.075
    counts=np.bincount(labels[hit],minlength=len(geoms))
    dominant=int(np.argmax(counts))
    wall_points=points[(labels==dominant)&hit]
    eigen=np.linalg.eigvalsh(np.cov(wall_points.T)) if len(wall_points)>2 else [0.,0.]
    ratio=float(eigen[0]/eigen[1]) if eigen[1]>0 else None
    result={"scan_id":scan["scan_id"],"time":scan["time"],
        "gt_base_pose":pose.tolist(),"total_bins":len(a),"valid_bins":int(valid.sum()),
        "invalid_or_empty_bins":int((~valid).sum()),
        "min_observed_range_m":float(a[valid].min()),
        "configured_range_min_m":scan["range_min"],
        "dominant_wall":geoms[dominant]["name"],
        "dominant_wall_rays":int(counts[dominant]),
        "dominant_fraction_valid":float(counts[dominant]/valid.sum()),
        "dominant_fraction_all_bins":float(counts[dominant]/len(a)),
        "dominant_wall_pca_minor_major_ratio":ratio,
        "unassociated_fraction_valid":float((~hit).mean()),
        "base_origin_to_world_surface_m":float(distance(pose[:2],geoms)),
        "actual_body_leg_clearance":clearance(scan["time"])}
    result["near_degenerate_single_wall"]=result["dominant_fraction_valid"]>=.70 and ratio is not None and ratio<=.01
    return result

scan_details={i:scan_geometry(s) for i,s in scans.items()}
edge_report=[]
for e in loop_edges:
    f=factor_index[frozenset((e["from_scan_id"],e["to_scan_id"]))]
    candidates=[event for event in accepted if event["scan_id"]==e["to_scan_id"]]
    event=min(candidates,key=lambda x:math.dist(x["candidate_map_pose"][:2],
        e["match_sensor_pose"][:2])) if candidates else None
    edge_report.append({"native":e,"factor":f,"acceptance":event,
        "from_scan":scan_details[e["from_scan_id"]],
        "to_scan":scan_details[e["to_scan_id"]]})
wrong=[e for e in edge_report if e["native"]["new_edge"] and not e["factor"]["correct"]]
normal_bad=[]
for e in native_edges:
    if e["new_edge"] and not e["loop"]:
        f=factor_index[frozenset((e["from_scan_id"],e["to_scan_id"]))]
        if not f["correct"]:
            normal_bad.append({"native":e,"factor":f})

yaw,tx,ty=run["saved_map_world_transform"]
pose=np.array([[float(r[k]) for k in ("time","x","y","yaw")] for r in rows("slam-pose.csv")])
gx=np.interp(pose[:,0],truth[:,0],truth[:,1])
gy=np.interp(pose[:,0],truth[:,0],truth[:,2])
expected=np.column_stack((math.cos(yaw)*gx-math.sin(yaw)*gy+tx,
                         math.sin(yaw)*gx+math.cos(yaw)*gy+ty))
error=np.linalg.norm(pose[:,1:3]-expected,axis=1)
crossings=np.flatnonzero(error>.3)
first_index=int(crossings[0]) if len(crossings) else None
first_time=float(pose[first_index,0]) if first_index is not None else None
worst_gap=None
first_max_gap=0.
with (ROOT/"driver-trace.jsonl").open() as f:
    for line in f:
        r=json.loads(line)
        if worst_gap is None or r["driver_loop_gap_wall_s"]>worst_gap["driver_loop_gap_wall_s"]:
            worst_gap=r
        if first_time is not None and r["time"]<=first_time:
            first_max_gap=max(first_max_gap,r["driver_loop_gap_wall_s"])
summary={"attempt":opt.attempt,
    "uncommitted_suffixes":uncommitted,
    "criteria":{"factor_xy_error_m":.10,"factor_yaw_error_deg":5.,
                "wall_endpoint_association_m":.075},
    "first_pose_error_over_0_3m":None if first_index is None else {
        "time":first_time,"error_m":float(error[first_index]),
        "previous_time":float(pose[first_index-1,0]) if first_index>0 else None,
        "previous_error_m":float(error[first_index-1]) if first_index>0 else None,
        "driver_max_gap_before_s":first_max_gap},
    "accepted_callbacks":len(accepted),"actual_loop_link_calls":len(loop_edges),
    "new_loop_factors":sum(e["native"]["new_edge"] for e in edge_report),
    "wrong_new_loop_factors":len(wrong),
    "wrong_normal_factors":len(normal_bad),
    "first_wrong_new_loop":wrong[0] if wrong else None,
    "first_wrong_normal_factor":normal_bad[0] if normal_bad else None,
    "normal_wrong_before_first_pose_jump":None if first_time is None else sum(
        e["native"]["to_time"]<first_time for e in normal_bad),
    "worst_driver_gap":worst_gap,
    "pose_error_m":{"max":float(error.max()),"p95":float(np.percentile(error,95))}}
(OUT/"summary.json").write_text(json.dumps(summary,indent=2))
(OUT/"loop-edges.json").write_text(json.dumps(edge_report,indent=2))
(OUT/"all-loop-decisions.json").write_text(json.dumps(parser.events,indent=2))
(OUT/"all-factor-errors.json").write_text(json.dumps(factors,indent=2))
with (OUT/"loop-edges.csv").open("w") as f:
    writer=csv.writer(f)
    writer.writerow(["time","from_id","to_id","new_edge","GT_distance_m",
        "factor_xy_error_m","factor_yaw_error_deg","correct","coarse","fine"])
    for r in edge_report:
        e,factor,event=r["native"],r["factor"],r["acceptance"]
        writer.writerow([e["to_time"],e["from_scan_id"],e["to_scan_id"],e["new_edge"],
            factor["gt_separation_m"],factor["relative_xy_error_m"],
            factor["relative_yaw_error_deg"],factor["correct"],
            event.get("coarse") if event else None,event.get("fine") if event else None])
fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True,constrained_layout=True)
axes[0].plot(pose[:,0],error,label="SLAM base vs GT")
axes[0].axhline(.1,color="black",linestyle="--")
axes[0].set_ylabel("XY error (m)")
for r in edge_report:
    color="green" if r["factor"]["correct"] else "red"
    axes[0].axvline(r["native"]["to_time"],color=color,alpha=.5,linewidth=.7)
    axes[1].scatter(r["native"]["to_time"],r["factor"]["relative_xy_error_m"],c=color)
axes[1].axhline(.1,color="black",linestyle="--")
axes[1].set_ylabel("Actual factor XY error (m)")
axes[1].set_xlabel("Simulation time (s)")
axes[0].set_title(f"Attempt {opt.attempt}: actual relative graph factors; red = wrong")
for ax in axes:
    ax.grid(alpha=.2)
fig.savefig(ROOT/"live"/f"attempt{opt.attempt}-error-vs-actual-edges.png",dpi=100)
print("EDGE_ANALYSIS "+json.dumps({k:v for k,v in summary.items()
    if k not in ("first_wrong_new_loop","first_wrong_normal_factor","worst_driver_gap")}))
print("FIRST_WRONG_LOOP "+json.dumps(summary["first_wrong_new_loop"]))
print("WORST_DRIVER_GAP "+json.dumps(worst_gap))
