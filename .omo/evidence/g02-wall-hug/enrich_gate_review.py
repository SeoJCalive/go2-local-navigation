"""Add a sensor-only line-support diagnostic to the frozen gate comparison."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/"offline-gate-review"
rows=json.loads((OUT/"all-accepted-edges.json").read_text())
sources={6:ROOT/"attempts/06/karto-scans.jsonl",
         7:OUT/"attempt-7/karto-scans.jsonl"}
geometry={}
for attempt,path in sources.items():
    wanted={r[key] for r in rows if r["attempt"]==attempt
            for key in ("from_id","to_id")}
    with path.open() as stream:
        for line in stream:
            if not line.endswith("\n"):
                break
            scan=json.loads(line)
            if scan["scan_id"] not in wanted:
                continue
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
            geometry[(attempt,scan["scan_id"])]=dict(
                dominant_line_fraction=support,min_range_m=float(ranges[valid].min()),
                all_endpoints_pca_ratio=float(eigen[0]/eigen[1]),
                near_wall_degenerate=bool(support>=.70 and ranges[valid].min()<=.50),
                rule="sensor-only: >=70% valid endpoints within 0.03m of one line; min range <=0.50m",
                hypotheses=2048,random_seed=20261125+scan["scan_id"])
for row in rows:
    row["from_geometry"]["sensor_only"]=geometry[(row["attempt"],row["from_id"])]
    row["to_geometry"]["sensor_only"]=geometry[(row["attempt"],row["to_id"])]
rejected=[r for r in rows if r["from_geometry"]["sensor_only"]["near_wall_degenerate"]
          or r["to_geometry"]["sensor_only"]["near_wall_degenerate"]]
summary=json.loads((OUT/"gate-comparison.json").read_text())
summary["sensor_only_degeneracy"]=dict(
    wrong_rejected=sum(not r["correct"] for r in rejected),
    correct_lost=sum(r["correct"] for r in rejected),
    rejected_edges=[f"{r['attempt']}:{r['from_id']}->{r['to_id']}" for r in rejected],
    maximum_line_support=max(g["dominant_line_fraction"] for g in geometry.values()),
    rule=">=70% valid endpoints within 0.03m of one line; min range <=0.50m")
(OUT/"all-accepted-edges.json").write_text(json.dumps(rows,indent=2))
(OUT/"gate-comparison.json").write_text(json.dumps(summary,indent=2))
print("SENSOR_GEOMETRY_REVIEW "+json.dumps(summary["sensor_only_degeneracy"]),flush=True)
