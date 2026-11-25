"""Freeze actual attempt-7 records and the GUI parameter source for analysis."""
import hashlib
import json
from pathlib import Path
import shutil

root=Path(__file__).resolve().parent
output=root/"offline-gate-review"
output.mkdir(exist_ok=True)
target=output/"attempt-7"
target.mkdir(exist_ok=True)
names=("result.json","physics-truth.csv","physics-contacts.csv","physics-geoms.csv",
       "slam-pose.csv","karto-scans.jsonl","karto-edges.jsonl","karto-candidates.jsonl",
       "driver-trace.jsonl","driver-result.json","g02-mapping.log",
       "attempt7-running-params.txt","route.json")
manifest={}
for name in names:
    source=root/name
    destination=target/name
    if destination.exists():
        raise RuntimeError("refuse to replace frozen evidence: "+str(destination))
    shutil.copy2(source,destination)
    raw=destination.read_bytes()
    manifest[name]=dict(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
source=root.parents[2]/"install/go2_nav2/share/go2_nav2/config/slam_mapping.yaml"
shutil.copy2(source,target/"gui-params-source.yaml")
manifest["gui-params-source.yaml"]=dict(
    source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest())
(target/"manifest.json").write_text(json.dumps(manifest,indent=2))
old=root/"live/attempt7-first-error-over-0.3m.json"
shutil.copy2(old,output/"attempt7-original-alert-stale-edge.json")
run=json.loads((target/"result.json").read_text())
assert run["attempt"]==7 and run["residue"]==[]
assert json.loads((root/"attempts/06/result.json").read_text())["attempt"]==6
print("GATE_REVIEW_FROZEN attempt6=attempts/06 attempt7=offline-gate-review/attempt-7",flush=True)
