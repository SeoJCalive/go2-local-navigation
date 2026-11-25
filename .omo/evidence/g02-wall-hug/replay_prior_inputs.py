"""Extract recorded query priors for diagnostic replay, never for live SLAM."""
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent
source=root/"karto-scans.jsonl"
data=source.read_bytes()
if not data.endswith(b"\n"):
    raise RuntimeError("baseline raw scan evidence has an unfinished record")
rows=[json.loads(line) for line in data.splitlines()]
output=root/"replay-baseline-priors.txt"
with output.open("w") as stream:
    for row in rows:
        stream.write(" ".join(map(str,[
            row["scan_id"],row["time"],*row["sensor_pose"]]))+"\n")
output.with_suffix(".provenance.json").write_text(json.dumps(dict(
    source=str(source),source_sha256=hashlib.sha256(data).hexdigest(),
    count=len(rows),purpose="Restore recorded pre-loop query hints in offline replay; "
    "does not supply GT or alter live SLAM"),indent=2))
print("REPLAY_PRIORS_SAVED",len(rows),flush=True)
