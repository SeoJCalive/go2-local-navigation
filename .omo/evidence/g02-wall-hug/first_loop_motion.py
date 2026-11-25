"""Read the frozen attempt-5 evidence for the five seconds before scan 278.

This cannot recover an unrecorded target node or raw scan. Those fields remain
unknown; a nearest historical pose is only a spatial reference, never an edge.
"""
import csv
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
CASE=ROOT/"diagnosis-5"
summary=json.loads((CASE/"summary.json").read_text())
event=summary["accepted"][0]
end=event["time"]
begin=end-5
rows=[]
with (CASE/"physics-truth.csv").open() as stream:
    reader=csv.DictReader(stream)
    for row in reader:
        stamp=float(row["time"])
        if begin<=stamp<=end:
            rows.append([float(row[key]) for key in (
                "time","x","y","yaw","clearance","closest_world","closest_robot")])
a=np.asarray(rows)
clear=a[:,4][np.isfinite(a[:,4])]
with (CASE/"physics-geoms.csv").open() as stream:
    names={int(row["id"]):row for row in csv.DictReader(stream)}
contacts=[]
with (CASE/"physics-contacts.csv").open() as stream:
    for row in csv.DictReader(stream):
        if begin<=float(row["time"])<=end:
            contacts.append(row)
trace=[json.loads(line) for line in (CASE/"driver-trace.jsonl").read_text().splitlines()]
last=max((row for row in trace if row["time"]<=end),key=lambda row:row["time"])
following=next(row for row in trace if row["time"]>end)
preceding=max((row for row in trace if row["time"]<=begin),key=lambda row:row["time"])
window=[preceding]+[row for row in trace if begin<row["time"]<=end]+[following]
durations=np.diff(a[:,0])
speeds=np.linalg.norm(np.diff(a[:,1:3],axis=0),axis=1)/durations
finite=a[np.isfinite(a[:,4])]
closest={}
for row in finite:
    name=names[int(row[5])]["name"]
    closest[name]=closest.get(name,0)+1
result={
    "scan_id":event["scan_id"],"time":end,
    "query_gt_world_xy":event["truth_base_world_xy"],
    "actual_target_node_id":None,"actual_target_gt_world_xy":None,
    "actual_node_separation_m":None,
    "target_status":"Not logged; nearest historical reference is not an actual edge.",
    "nearest_historical_reference_not_edge":event["nearest_gt_prior_node_not_actual_edge"],
    "actual_scan_content":{
        "recorded":False,"dominant_nearby_wall_ray_fraction":None,
        "observed_min_range_m":None,"straight_wall_degeneracy":None,
        "reason":"No raw LaserScan or Karto point-reading snapshot was saved in attempt 5."},
    "preceding_five_seconds":{
        "begin":begin,"end":end,"sample_count":len(a),
        "start_world_xy":a[0,1:3].tolist(),"end_world_xy":a[-1,1:3].tolist(),
        "distance_travelled_m":float(np.linalg.norm(np.diff(a[:,1:3],axis=0),axis=1).sum()),
        "yaw_change_rad":float(np.unwrap(a[:,3])[-1]-np.unwrap(a[:,3])[0]),
        "speed_mps":{"median":float(np.median(speeds)),"max":float(speeds.max())},
        "analytic_envelope_clearance_m":{"median":float(np.median(clear)),
            "p10":float(np.percentile(clear,10)),"min":float(clear.min())},
        "nearest_world_sample_counts":closest,
        "nonfoot_contact_points":len(contacts),
        "maximum_contact_depth_m":max([-float(row["dist"]) for row in contacts],default=0),
        "contacts":contacts,
        "commands":window,
        "held_command_at_false_loop":{
            "from":last["time"],"next_update":following["time"],
            "hold_gap_s":following["time"]-last["time"],"command":last["command"],
            "cross_track_before_m":last["cross_track_m"],
            "cross_track_after_m":following["cross_track_m"]}},
    "loop_parameters_in_use":{
        "loop_search_maximum_distance":3.0,"loop_match_minimum_chain_size":10,
        "loop_match_minimum_response_coarse":.35,"loop_match_minimum_response_fine":.45,
        "loop_match_maximum_variance_coarse":3.0,"loop_search_space_dimension":8.0},
    "accepted_match":{"coarse_response":.378986,"coarse_variance_xy":[.00263862,.00263862],
        "fine_response":.472931,"candidate_base_pose":event["candidate_map_pose"],
        "candidate_gt_xy_error_m":event["candidate_xy_error_m"],
        "slam_xy_error_before_m":event["xy_error_before_m"],
        "slam_xy_error_after_m":event["xy_error_after_m"]},
    "causal_status":"False loop and capture-blocked command overlap. Independence is unproven. "
        "Keep SLAM parameters unchanged for the non-blocking same-route run."}
(CASE/"first-loop-details.json").write_text(json.dumps(result,indent=2))
fig,axes=plt.subplots(1,2,figsize=(12,5))
axes[0].scatter(a[:,1],a[:,2],c=a[:,0],s=5,cmap="viridis")
axes[0].plot([a[0,1],a[-1,1]],[a[0,2],a[0,2]],"--",label="start lateral reference")
axes[0].scatter([a[-1,1]],[a[-1,2]],marker="x",color="red",label="false loop")
axes[0].set_aspect("equal",adjustable="datalim")
axes[0].set_xlabel("World x (m)")
axes[0].set_ylabel("World y (m)")
axes[0].legend()
axes[1].plot(finite[:,0],finite[:,4],label="analytic body/leg envelope")
axes[1].axhline(.10,ls="--",color="black",label="0.10 m")
axes[1].axvline(last["time"],color="orange",label="held command starts")
axes[1].set_xlabel("Simulator time (s)")
axes[1].set_ylabel("Conservative 2D clearance (m)")
axes[1].legend()
for axis in axes:
    axis.grid(alpha=.25)
fig.suptitle("Scan 278: actual qpos motion during the preceding five seconds")
fig.tight_layout()
fig.savefig(ROOT/"live/partial-5-first-loop-motion.png",dpi=100)
print(json.dumps(result,indent=2))
