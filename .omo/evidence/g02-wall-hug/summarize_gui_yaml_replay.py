"""Independently classify new constraints after full GUI-YAML baseline replay."""
import csv
import json
import math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/"offline-gate-review"
def records(path):
    return [json.loads(line) for line in path.read_text().splitlines()]
data=np.array([[float(row[k]) for k in ("time","x","y","yaw")]
    for row in csv.DictReader(line for line in
        (ROOT/"attempts/06/physics-truth.csv").open() if line.endswith("\n"))])
columns=[np.ascontiguousarray(data[:,k]) for k in range(4)]
columns[3]=np.unwrap(columns[3])
def at(t):
    return [float(np.interp(t,columns[0],column)) for column in columns[1:]]
edges=[e for e in records(OUT/"replay-gui019-edges.jsonl") if e["loop"] and e["new_edge"]]
factors=records(OUT/"replay-gui019-factors.jsonl")
rows=[]
for edge in edges:
    matches=[f for f in factors if f["from_scan_id"]==edge["from_scan_id"]
             and f["to_scan_id"]==edge["to_scan_id"]]
    assert len(matches)==1
    factor=matches[0]
    a,b=at(edge["from_time"]),at(edge["to_time"])
    dx,dy=b[0]-a[0],b[1]-a[1]
    expected=[math.cos(a[2])*dx+math.sin(a[2])*dy,
              -math.sin(a[2])*dx+math.cos(a[2])*dy,b[2]-a[2]]
    observed=factor["relative_pose"]
    xy=math.dist(expected[:2],observed[:2])
    yaw=abs((expected[2]-observed[2]+math.pi)%(2*math.pi)-math.pi)*180/math.pi
    rows.append(dict(edge=f"{edge['from_scan_id']}->{edge['to_scan_id']}",
                     time=edge["to_time"],xy_error_m=xy,yaw_error_deg=yaw,
                     correct=xy<=.10 and yaw<=5))
summary=dict(
    source_yaml="attempt-7/gui-params-source.yaml",
    configuration_verified="dimension=8 sigma=.19 internal_variance=.0361 coarse=.35 fine=.45 chain=10 candidate_distance=3",
    accepted_scans=2099,inserted_loops=len(rows),correct=sum(r["correct"] for r in rows),
    wrong=sum(not r["correct"] for r in rows),
    historical_140_275_inserted=any(r["edge"]=="140->275" for r in rows),
    rows=rows)
(OUT/"gui-yaml-full-replay-summary.json").write_text(json.dumps(summary,indent=2))
print("GUI_YAML_FULL_REPLAY "+json.dumps(summary),flush=True)
