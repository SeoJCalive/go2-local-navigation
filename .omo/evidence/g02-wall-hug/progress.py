"""Append non-GUI diagnosis/restart milestones to the owner's live ledger."""
from datetime import datetime
from pathlib import Path
import sys

root=Path(__file__).resolve().parent
line=f"{datetime.now():%H:%M:%S} | {sys.argv[1]} | {sys.argv[2]} | {sys.argv[3] if len(sys.argv)>3 else '-'}"
with (root/"live/progress.log").open("a") as stream:
    stream.write(line+"\n")
print(line,flush=True)
