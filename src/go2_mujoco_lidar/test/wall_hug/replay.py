"""Replay recorded G02 loop observations and compare frozen-candidate gates.

Promoted numerical algorithms from compare_recorded_gates.py/enrich_gate_review.py
(123f8b7); raw edge/candidate/scan streams and qpos/serialized odom produce measured
relative-pose errors, covariance/chain checks and a comparison table. CLI outputs
JSON with runtime. No ROS/MuJoCo imports. See README.md and G02_WALL_HUG.md.
Attempt-6 candidate chains came from recorded-prior engine replay; attempt-7
odometry/fine covariance were not saved. Gate replay does not optimize a new graph.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np

ROOT=Path(__file__).resolve().parent
DEFAULT_DATASET=Path("/home/bi-agx1/go2_projects/projects/go2_local_navigation/.omo/evidence/g02-wall-hug")


class DatasetAbsent(FileNotFoundError):
    pass


def checked_dataset(root, manifest=None):
    manifest=manifest or json.loads((ROOT/"fixtures/dataset.json").read_text())
    missing=[str(root/name) for name in manifest if not (root/name).is_file()]
    if missing:
        raise DatasetAbsent("G02 recorded dataset absent; pinned AGX path: "+str(DEFAULT_DATASET)+"; missing: "+", ".join(missing))
    for name,expected in manifest.items():
        digest=hashlib.sha256()
        with (root/name).open("rb") as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b""):
                digest.update(chunk)
        if digest.hexdigest()!=expected:
            raise ValueError("G02 SHA256 mismatch: "+str(root/name))


def records(path):
    with path.open() as stream:
        for line in stream:
            if not line.endswith("\n"):
                raise ValueError("incomplete recorded JSONL: "+str(path))
            yield json.loads(line)


def truth(path):
    with path.open() as stream:
        # Native flush can leave a final partial frame on owner-requested stop.
        complete=(line for line in stream if line.endswith("\n"))
        data=np.array([[float(r[k]) for k in ("time","x","y","yaw")] for r in csv.DictReader(complete)])
    data[:,3]=np.unwrap(data[:,3])
    return data


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


def line_support(scan):
    ranges=np.array([np.nan if r is None else r for r in scan["ranges"]])
    valid=np.isfinite(ranges)&(ranges>=scan["range_min"])&(ranges<=scan["range_max"])
    angles=scan["angle_min"]+np.arange(len(ranges))*scan["angle_increment"]
    points=np.column_stack((ranges[valid]*np.cos(angles[valid]),
                            ranges[valid]*np.sin(angles[valid])))
    # Deterministic hypotheses from the actual endpoints. No GT, map,
    # simulator pose, or learned threshold enters this classifier.
    rng=np.random.default_rng(20261125+scan["scan_id"])
    pairs=rng.integers(0,len(points),size=(2048,2))
    a=points[pairs[:,0]]; d=points[pairs[:,1]]-a
    usable=np.linalg.norm(d,axis=1)>=.30
    a=a[usable];d=d[usable]
    normals=np.column_stack((-d[:,1],d[:,0]))
    normals/=np.linalg.norm(normals,axis=1)[:,None]
    offsets=np.einsum("ij,ij->i",normals,a)
    errors=np.abs(points@normals.T-offsets[None,:])
    index=int(np.argmax(np.sum(errors<=.03,axis=0)))
    support=float(np.mean(errors[:,index]<=.03))
    eigen=np.linalg.eigvalsh(np.cov(points.T))
    return dict(
        dominant_line_fraction=support,min_range_m=float(ranges[valid].min()),
        all_endpoints_pca_ratio=float(eigen[0]/eigen[1]),
        near_wall_degenerate=bool(support>=.70 and ranges[valid].min()<=.50),
        rule="sensor-only: >=70% valid endpoints within 0.03m of one line; min range <=0.50m",
        hypotheses=2048,random_seed=20261125+scan["scan_id"])


def candidate_for(edge, candidates):
    matches=[c for c in candidates if c["scan_id"]==edge["to_scan_id"]
        and max(abs(a-b) for a,b in zip(c["fine_sensor_pose"],edge["match_sensor_pose"]))<=1e-6]
    if len(matches)!=1:
        raise ValueError("edge must match one recorded candidate")
    c=matches[0]
    if len(c["coarse_covariance"])!=9 or c["chain_size"]!=len(c["chain"]):
        raise ValueError("invalid covariance or chain size")
    if len({p["scan_id"] for p in c["chain"]})!=c["chain_size"]:
        raise ValueError("duplicate chain scan IDs")
    anchor=[p for p in c["chain"] if p["scan_id"]==edge["from_scan_id"]]
    if len(anchor)!=1:
        raise ValueError("actual edge anchor absent from chain")
    return c,anchor[0]["pose"]


def classify(rows):
    """Recompute correctness from immutable relative measurement versus qpos GT."""
    result=[]
    for row in rows:
        row=dict(row)
        expected=relative(np.array(row["gt_from_pose"]),np.array(row["gt_to_pose"]))
        measured=np.array(row["relative_pose"])
        xy=float(np.linalg.norm(measured[:2]-expected[:2]))
        yaw=math.degrees(abs(wrap(measured[2]-expected[2])))
        row.update(relative_xy_error_m=xy,relative_yaw_error_deg=yaw,correct=xy<=.10 and yaw<=5.)
        result.append(row)
    return result


def load_dataset(root):
    checked_dataset(root)
    rows=[]
    factors={r["native"]["to_scan_id"]:r["factor"] for r in json.loads(
        (root/"analysis-6/loop-edges.json").read_text())}
    for attempt,base,edges_path,candidates_path in (
        (6,root/"attempts/06",root/"attempts/06/karto-edges.jsonl",root/"replay-restored-candidates.jsonl"),
        (7,root/"offline-gate-review/attempt-7",root/"offline-gate-review/attempt-7/karto-edges.jsonl",root/"offline-gate-review/attempt-7/karto-candidates.jsonl")):
        data=truth(base/"physics-truth.csv")
        edges=[e for e in records(edges_path) if e["loop"] and e["new_edge"]]
        candidates=list(records(candidates_path))
        raw=list(records(base/"karto-scans.jsonl"))
        scans={s["scan_id"]:s for s in raw}
        if len(scans)!=len(raw):
            raise ValueError("duplicate raw scan IDs")
        odom=truth(root/"offline-gate-review/attempt6-serialized-odom.csv") if attempt==6 else np.array(
            [[scan["time"],*at(data,scan["time"])] for scan in sorted(raw,key=lambda s:s["time"])])
        cumulative=traveled(odom)
        def path_at(t):
            return float(np.interp(t,odom[:,0],cumulative))
        previous=0.
        for edge in edges:
            c,anchor=candidate_for(edge,candidates)
            pair=[scans[edge[key]] for key in ("from_scan_id","to_scan_id")]
            for scan,stamp in zip(pair,(edge["from_time"],edge["to_time"])):
                if abs(scan["time"]-stamp)>1e-6 or len(scan["ranges"])!=720:
                    raise ValueError("raw scan identity/stamp mismatch")
                if not data[0,0]<=stamp<=data[-1,0]:
                    raise ValueError("GT does not bracket scan")
            a,b=at(data,edge["from_time"]),at(data,edge["to_time"])
            measured=relative(np.array(anchor),np.array(c["fine_sensor_pose"]))
            fine_covariance=None
            if attempt==6:
                factor=factors[edge["to_scan_id"]]
                fine_covariance=factor["covariance"]
                if len(fine_covariance)!=9 or max(abs(a-b) for a,b in zip(
                        measured,factor["relative_pose"]))>1e-6:
                    raise ValueError("saved LinkInfo differs from recorded candidate measurement")
            arc=path_at(edge["to_time"])
            row=dict(attempt=attempt,from_id=edge["from_scan_id"],to_id=edge["to_scan_id"],
                time=edge["to_time"],gt_from_pose=a.tolist(),gt_to_pose=b.tolist(),
                relative_pose=measured.tolist(),coarse=c["coarse_response"],fine=c["fine_response"],
                coarse_covariance=c["coarse_covariance"],chain_size=c["chain_size"],
                fine_link_covariance=fine_covariance,
                implied_correction_m=math.dist(c["fine_sensor_pose"][:2],c["prior_sensor_pose"][:2]),
                path_cumulative_m=arc,distance_since_actual_loop_m=arc-previous,
                from_geometry={"sensor_only":line_support(pair[0])},
                to_geometry={"sensor_only":line_support(pair[1])})
            rows.append(row)
            previous=arc
    return classify(rows)


def evaluate(rows,name,coarse=.35,fine=.45,sigma=3.,chain=10,motion=None,
             reset="actual"):
    rejected=[]
    kept_anchor={attempt:0. for attempt in (6,7)}
    for row in rows:
        allowed=(row["coarse"]>coarse and row["fine"]>=fine
                 and max(row["coarse_covariance"][0],row["coarse_covariance"][4])<sigma*sigma
                 and row["chain_size"]>=chain)
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
        wrong_rejected=sum(not row["correct"] for row in rejected),
        wrong_accepted=sum(not r["correct"] for r in rows)-sum(not row["correct"] for row in rejected),
        correct_lost=sum(row["correct"] for row in rejected),
        correct_retained=sum(r["correct"] for r in rows)-sum(row["correct"] for row in rejected),
        rejected_edges=[f"{r['attempt']}:{r['from_id']}->{r['to_id']}" for r in rejected])

def comparison(rows):
    profiles=[evaluate(rows,"original"),evaluate(rows,"sigma .19",sigma=.19),
        evaluate(rows,"coarse .47",coarse=.47),evaluate(rows,"fine .50",fine=.50),
        evaluate(rows,"chain 20",chain=20),evaluate(rows,"chain 40",chain=40),
        evaluate(rows,"coarse .42 sigma .19",coarse=.42,sigma=.19),
        evaluate(rows,"chain 20 sigma .19",chain=20,sigma=.19),
        evaluate(rows,"fine .48 sigma .19",fine=.48,sigma=.19)]
    for k in (.02,.05,.10):
        profiles.append(evaluate(rows,f"motion {k:.0%}+.20",motion=(k,.20),reset="retained"))
        profiles.append(evaluate(rows,f"motion {k:.0%}+.20 sigma .19",motion=(k,.20),sigma=.19,reset="retained"))
    for limit in (.70,.35,.40,.45,.50,.55,.60):
        rejected=[r for r in rows if any(r[side]["sensor_only"]["dominant_line_fraction"]>=limit
            and r[side]["sensor_only"]["min_range_m"]<=.50 for side in ("from_geometry","to_geometry"))]
        profiles.append(dict(name=f"line {limit:.0%}",wrong_rejected=sum(not r["correct"] for r in rejected),
            correct_lost=sum(r["correct"] for r in rejected)))
    wrong6=[r for r in rows if r["attempt"]==6 and not r["correct"]]
    first=min(wrong6,key=lambda r:r["time"])
    return dict(attempt6_inserted=sum(r["attempt"]==6 for r in rows),attempt6_wrong=len(wrong6),
        first_wrong_edge=[first["from_id"],first["to_id"]],first_wrong_time_s=first["time"],
        profiles=profiles)


EXPECTED_COUNTS={
    "original":(0,0),"sigma .19":(5,2),"coarse .47":(6,7),"fine .50":(4,6),
    "chain 20":(2,4),"chain 40":(5,7),"coarse .42 sigma .19":(6,5),
    "chain 20 sigma .19":(6,5),"fine .48 sigma .19":(6,4),
    "motion 2%+.20":(6,3),"motion 2%+.20 sigma .19":(6,5),
    "motion 5%+.20":(6,3),"motion 5%+.20 sigma .19":(6,4),
    "motion 10%+.20":(3,4),"motion 10%+.20 sigma .19":(5,4),
    "line 70%":(0,0),"line 35%":(6,6),"line 40%":(4,4),"line 45%":(2,2),
    "line 50%":(2,1),"line 55%":(0,1),"line 60%":(0,0)}


def verify(summary):
    assert summary["attempt6_inserted"]==15
    assert summary["attempt6_wrong"]==5
    assert summary["first_wrong_edge"]==[140,275]
    assert summary["first_wrong_time_s"]==201.302
    actual={p["name"]:(p["wrong_rejected"],p["correct_lost"]) for p in summary["profiles"]}
    assert actual==EXPECTED_COUNTS,(actual,EXPECTED_COUNTS)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset",type=Path,default=DEFAULT_DATASET)
    parser.add_argument("--fixture",action="store_true",help="use small committed measurement projection")
    parser.add_argument("--export-measurements",action="store_true",help="print measured fixture inputs, no file writes")
    args=parser.parse_args()
    began=time.perf_counter()
    try:
        rows=classify(json.loads((ROOT/"fixtures/measurements.json").read_text())) if args.fixture else load_dataset(args.dataset)
    except DatasetAbsent as error:
        print("SKIP: "+str(error))
        raise SystemExit(77)
    if args.export_measurements:
        for r in rows:
            for key in ("correct","relative_xy_error_m","relative_yaw_error_deg"):
                del r[key]
        print(json.dumps(rows,separators=(",",":")))
    else:
        summary=comparison(rows)
        verify(summary)
        summary["runtime_s"]=time.perf_counter()-began
        summary["mode"]="committed measurement fixture" if args.fixture else "SHA256 checked recorded dataset; fixed candidates, no graph optimization"
        print(json.dumps(summary,indent=2))
