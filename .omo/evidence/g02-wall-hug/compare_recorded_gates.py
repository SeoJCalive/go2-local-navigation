"""Replay acceptance gates over frozen, actually accepted loop candidates.

This does not re-optimize a counterfactual graph or implement a product gate.
Attempt 6 uses serialized actual odometry. Attempt 7 did not serialize odometry;
its distance is explicitly a qpos-at-scan-stamps proxy, never labeled recorded
odometry. Original and gate-retained loop-anchor policies are reported separately.
"""
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import numpy as np
from geometry import obstacles,distance

ROOT=Path(__file__).resolve().parent
OUT=ROOT/"offline-gate-review"
def load(path):
    return json.loads(path.read_text())
def lines(path):
    with path.open() as stream:
        for line in stream:
            if line.endswith("\n"):
                yield line
def records(path):
    return [json.loads(line) for line in lines(path)]
def truth(path):
    return np.array([[float(row[k]) for k in ("time","x","y","yaw","clearance")]
                     for row in csv.DictReader(lines(path))])
def wrap(a):
    return (a+math.pi)%(2*math.pi)-math.pi
def at(data,t):
    value=np.array([np.interp(t,data[:,0],data[:,k]) for k in (1,2,3)])
    value[2]=wrap(value[2])
    return value
def relative(a,b):
    dx,dy=b[:2]-a[:2]
    return np.array([math.cos(a[2])*dx+math.sin(a[2])*dy,
                     -math.sin(a[2])*dx+math.cos(a[2])*dy,wrap(b[2]-a[2])])
def traveled(samples):
    return np.r_[0.,np.cumsum(np.linalg.norm(np.diff(samples[:,1:3],axis=0),axis=1))]
world=ROOT/"analysis-6/world.xml"
geoms=obstacles(world)
truth6=truth(ROOT/"attempts/06/physics-truth.csv")
truth7=truth(OUT/"attempt-7/physics-truth.csv")
for data in (truth6,truth7):
    data[:,3]=np.unwrap(data[:,3])
odom6=np.array([[float(r[k]) for k in ("time","x","y","yaw")]
                for r in csv.DictReader((OUT/"attempt6-serialized-odom.csv").open())])
raw7={row["scan_id"]:row for row in records(OUT/"attempt-7/karto-scans.jsonl")}
odom7=np.array([[scan["time"],*at(truth7,scan["time"])]
                for scan in sorted(raw7.values(),key=lambda row:row["time"])])
paths={6:(odom6,traveled(odom6)),7:(odom7,traveled(odom7))}
def path_at(attempt,t):
    samples,cumulative=paths[attempt]
    return float(np.interp(t,samples[:,0],cumulative))
def scan_geometry(scan,data):
    ranges=np.array([np.nan if x is None else x for x in scan["ranges"]])
    valid=np.isfinite(ranges)&(ranges>=scan["range_min"])&(ranges<=scan["range_max"])
    angles=scan["angle_min"]+np.arange(len(ranges))*scan["angle_increment"]
    pose=at(data,scan["time"])
    points=np.column_stack((
        pose[0]+ranges[valid]*np.cos(angles[valid]+pose[2]),
        pose[1]+ranges[valid]*np.sin(angles[valid]+pose[2])))
    residuals=np.array([np.abs(distance(points,[geom])) for geom in geoms])
    best=np.argmin(residuals,axis=0)
    hit=np.min(residuals,axis=0)<=.075
    counts=np.bincount(best[hit],minlength=len(geoms))
    dominant=int(np.argmax(counts))
    selected=points[hit&(best==dominant)]
    eigen=np.linalg.eigvalsh(np.cov(selected.T))
    wall_ratio=float(eigen[0]/eigen[1])
    all_eigen=np.linalg.eigvalsh(np.cov(points.T))
    all_ratio=float(all_eigen[0]/all_eigen[1])
    fraction=float(counts[dominant]/valid.sum())
    near=bool(fraction>=.70 and wall_ratio<=.01)
    sample=data[np.argmin(abs(data[:,0]-scan["time"]))]
    return dict(valid_rays=int(valid.sum()),total_rays=len(ranges),
        min_range_m=float(ranges[valid].min()),dominant_wall=geoms[dominant]["name"],
        dominant_wall_fraction_valid=fraction,dominant_wall_pca_ratio=wall_ratio,
        all_endpoints_pca_ratio=all_ratio,near_wall_degenerate_gt_association=near,
        body_leg_clearance_m=float(sample[4]) if np.isfinite(sample[4]) else None,
        rule="dominant physical wall >=70% valid endpoints and its PCA ratio <=0.01")
base=load(ROOT/"analysis-6/loop-edges.json")
candidate6=records(ROOT/"replay-restored-candidates.jsonl")
candidate7=records(OUT/"attempt-7/karto-candidates.jsonl")
edges7=[row for row in records(OUT/"attempt-7/karto-edges.jsonl")
        if row["loop"] and row["new_edge"]]
rows=[]
for attempt,edges,candidates,data in (
        (6,[row["native"] for row in base],candidate6,truth6),
        (7,edges7,candidate7,truth7)):
    previous_time=float(paths[attempt][0][0,0])
    for edge in edges:
        matches=[candidate for candidate in candidates
                 if candidate["scan_id"]==edge["to_scan_id"]
                 and max(abs(a-b) for a,b in zip(
                     candidate["fine_sensor_pose"],edge["match_sensor_pose"]))<=1e-6]
        assert len(matches)==1,(attempt,edge,len(matches))
        candidate=matches[0]
        anchor=next(row for row in candidate["chain"]
                    if row["scan_id"]==edge["from_scan_id"])["pose"]
        prior=np.array(candidate["prior_sensor_pose"])
        fine=np.array(candidate["fine_sensor_pose"])
        a,b=at(data,edge["from_time"]),at(data,edge["to_time"])
        measured=relative(np.array(anchor),fine)
        expected=relative(a,b)
        xy=float(np.linalg.norm(measured[:2]-expected[:2]))
        yaw=math.degrees(abs(wrap(measured[2]-expected[2])))
        correct=xy<=.10 and yaw<=5.
        if attempt==6:
            observed=next(row for row in base if row["native"]["to_scan_id"]==edge["to_scan_id"])
            assert abs(xy-observed["factor"]["relative_xy_error_m"])<1e-6
            assert correct==observed["factor"]["correct"]
            from_scan=observed["from_scan"]; to_scan=observed["to_scan"]
            def original_geometry(scan):
                return dict(
                    valid_rays=scan["valid_bins"],total_rays=scan["total_bins"],
                    min_range_m=scan["min_observed_range_m"],
                    dominant_wall=scan["dominant_wall"],
                    dominant_wall_fraction_valid=scan["dominant_fraction_valid"],
                    dominant_wall_pca_ratio=scan["dominant_wall_pca_minor_major_ratio"],
                    near_wall_degenerate_gt_association=scan["near_degenerate_single_wall"],
                    body_leg_clearance_m=scan["actual_body_leg_clearance"].get("clearance_m"),
                    all_endpoints_pca_ratio=None)
            from_geometry=original_geometry(from_scan)
            to_geometry=original_geometry(to_scan)
            fine_covariance=observed["factor"]["covariance"]
        else:
            from_geometry=scan_geometry(raw7[edge["from_scan_id"]],data)
            to_geometry=scan_geometry(raw7[edge["to_scan_id"]],data)
            fine_covariance=None
        row=dict(attempt=attempt,from_id=edge["from_scan_id"],to_id=edge["to_scan_id"],
            from_time=edge["from_time"],time=edge["to_time"],correct=correct,
            relative_xy_error_m=xy,relative_yaw_error_deg=yaw,
            coarse=candidate["coarse_response"],fine=candidate["fine_response"],
            coarse_covariance=candidate["coarse_covariance"],
            fine_link_covariance=fine_covariance,
            fine_covariance_provenance="actual saved LinkInfo" if attempt==6 else
                "not serialized on owner-requested stop; not reconstructed",
            chain_size=candidate["chain_size"],
            candidate_provenance="recorded-hint replay, all 15 actual pairs/poses reproduced <=1.21e-8"
                if attempt==6 else "actual native candidate record",
            gt_node_distance_m=float(np.linalg.norm(b[:2]-a[:2])),
            estimated_node_distance_before_match_m=float(np.linalg.norm(
                prior[:2]-np.array(anchor[:2]))),
            estimated_node_distance_at_link_m=float(np.linalg.norm(
                fine[:2]-np.array(anchor[:2]))),
            implied_correction_m=float(np.linalg.norm(fine[:2]-prior[:2])),
            implied_yaw_correction_deg=math.degrees(abs(wrap(fine[2]-prior[2]))),
            previous_actual_loop_time=previous_time,
            distance_since_actual_loop_m=path_at(attempt,edge["to_time"])-path_at(attempt,previous_time),
            path_cumulative_m=path_at(attempt,edge["to_time"]),
            distance_source="serialized actual scan odometry" if attempt==6 else
                "qpos at actual raw-scan stamps; odometry was not recorded",
            from_geometry=from_geometry,to_geometry=to_geometry)
        rows.append(row)
        previous_time=edge["to_time"]
assert len(rows)==16 and sum(row["correct"] for row in rows)==10
def evaluate(name,coarse=.35,fine=.45,sigma=3.,chain=10,motion=None,degenerate=False,
             reset="actual"):
    rejected=[]
    kept_anchor={attempt:0. for attempt in (6,7)}
    minimum_margin=None
    for row in rows:
        allowed=(row["coarse"]>coarse and row["fine"]>=fine
                 and max(row["coarse_covariance"][0],row["coarse_covariance"][4])<sigma*sigma
                 and row["chain_size"]>=chain)
        if degenerate:
            allowed=allowed and not (
                row["from_geometry"]["near_wall_degenerate_gt_association"]
                or row["to_geometry"]["near_wall_degenerate_gt_association"])
        if motion:
            k,c=motion
            traveled_m=(row["distance_since_actual_loop_m"] if reset=="actual" else
                        row["path_cumulative_m"]-kept_anchor[row["attempt"]])
            bound=k*traveled_m+c
            allowed=allowed and row["implied_correction_m"]<=bound
        if allowed:
            kept_anchor[row["attempt"]]=row["path_cumulative_m"]
        else:
            rejected.append(row)
    return dict(name=name,coarse=coarse,fine=fine,sigma=sigma,
        internal_variance_limit_m2=sigma*sigma,chain_min=chain,
        motion_fraction=motion[0] if motion else None,
        motion_constant_m=motion[1] if motion else None,anchor_policy=reset,
        exclude_degenerate=degenerate,
        wrong_rejected=sum(not row["correct"] for row in rejected),
        wrong_accepted=6-sum(not row["correct"] for row in rejected),
        correct_lost=sum(row["correct"] for row in rejected),
        correct_retained=10-sum(row["correct"] for row in rejected),
        rejected_edges=[f"{r['attempt']}:{r['from_id']}->{r['to_id']}" for r in rejected])
profiles=[
    evaluate("original"),
    evaluate("GUI sigma .19",sigma=.19),
    evaluate("response coarse .47",coarse=.47),
    evaluate("fine .50",fine=.50),
    evaluate("chain >=20",chain=20),
    evaluate("chain >=40",chain=40),
    evaluate("coarse .42 + sigma .19",coarse=.42,sigma=.19),
    evaluate("chain >=20 + sigma .19",chain=20,sigma=.19),
    evaluate("near-wall-degenerate excluded",degenerate=True),
]
for k,c in itertools.product((.02,.05,.10),(.10,.20)):
    for reset in ("actual","retained"):
        profiles.append(evaluate(f"motion {k:.0%}+. {c:.2f}m",motion=(k,c),reset=reset))
        profiles.append(evaluate(f"motion {k:.0%}+{c:.2f}m +sigma .19",
                                 motion=(k,c),sigma=.19,reset=reset))
        profiles.append(evaluate(f"motion {k:.0%}+{c:.2f}m +coarse .40",
                                 motion=(k,c),coarse=.40,reset=reset))
grid=[]
for coarse,fine,sigma,chain in itertools.product(
        (.35,.37,.38,.39,.40,.41,.42,.425,.43,.44,.45,.47,.50,.55,.60),
        (.45,.46,.47,.48,.49,.50,.51,.52,.525,.55,.60),
        (.15,.18,.19,.195,.20,.22,.24,.25,.30,.5,3.),
        (10,12,15,18,20,25,30,35,40,50,60,75,80)):
    result=evaluate("config grid",coarse,fine,sigma,chain)
    if result["wrong_accepted"]==0:
        grid.append(result)
grid.sort(key=lambda r:(r["correct_lost"],r["chain_min"],r["coarse"],r["fine"]))
dominance=[]
for bad in (row for row in rows if not row["correct"]):
    for good in (row for row in rows if row["correct"]):
        if (bad["coarse"]>=good["coarse"] and bad["fine"]>=good["fine"]
            and bad["chain_size"]>=good["chain_size"]
            and max(bad["coarse_covariance"][0],bad["coarse_covariance"][4])
                <=max(good["coarse_covariance"][0],good["coarse_covariance"][4])):
            dominance.append(dict(wrong=f"{bad['attempt']}:{bad['from_id']}->{bad['to_id']}",
                                  correct=f"{good['attempt']}:{good['from_id']}->{good['to_id']}"))
q6=np.array([[row[0],*at(truth6,row[0])] for row in odom6])
summary=dict(observed_edges=16,wrong=6,correct=10,
    evaluation="Frozen accepted-candidate trace replay; not a counterfactual optimized graph",
    odom6_distance_m=float(paths[6][1][-1]),
    qpos6_distance_at_same_stamps_m=float(traveled(q6)[-1]),
    attempt7_distance_limitation="No serialized odometry; qpos-stamp proxy only",
    fine_covariance_missing_attempt7=True,profiles=profiles,
    best_config_grid=grid[:25],monotone_threshold_dominance=dominance)
(OUT/"all-accepted-edges.json").write_text(json.dumps(rows,indent=2))
(OUT/"gate-comparison.json").write_text(json.dumps(summary,indent=2))
columns=("attempt","from_id","to_id","time","correct","coarse","fine","chain_size",
         "gt_node_distance_m","estimated_node_distance_before_match_m",
         "estimated_node_distance_at_link_m","implied_correction_m",
         "implied_yaw_correction_deg","distance_since_actual_loop_m","distance_source")
with (OUT/"all-accepted-edges.csv").open("w") as stream:
    writer=csv.DictWriter(stream,fieldnames=columns,extrasaction="ignore")
    writer.writeheader();writer.writerows(rows)
print("RECORDED_GATE_COMPARISON "+json.dumps(dict(
    edges=16,wrong=6,correct=10,best_config=grid[:1],
    profiles=[{k:p[k] for k in ("name","anchor_policy","wrong_rejected","correct_lost")}
              for p in profiles],dominance=dominance)),flush=True)
