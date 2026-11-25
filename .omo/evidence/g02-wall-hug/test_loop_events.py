"""Independent accepted, coarse-rejected and fine-rejected log controls."""
import ast
from pathlib import Path
import re

tree=ast.parse(Path(__file__).with_name("gui_run.py").read_text())
definition=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=="LoopEvents")
scope={"re":re}
exec(compile(ast.Module(body=[definition],type_ignores=[]),"LoopEvents","exec"),scope)
events=scope["LoopEvents"]()
lines=[
    "BEST POSE = 1.0 2.0 0.1 BEST RESPONSE = 0.9",
    "G02_LOOP event=check scan=1 time=2.000000000 info=COARSE RESPONSE: .9 (> .35) var: 1, 1",
    "BEST POSE = 1.1 2.1 0.2 BEST RESPONSE = 0.8",
    "G02_LOOP event=check scan=1 time=2.000000000 info=FINE RESPONSE: .8 (> .45)",
    "G02_LOOP event=accepted scan=1 time=2.000000000 info=Loop closed!",
    "G02_LOOP event=scan_end scan=1 time=2.000000000 info=accepted",
    "BEST POSE = 3.0 4.0 0.1 BEST RESPONSE = 0.1",
    "G02_LOOP event=check scan=2 time=3.000000000 info=COARSE RESPONSE: .1 (> .35) var: 4, 4",
    "G02_LOOP event=scan_end scan=2 time=3.000000000 info=no_accepted_closure",
    "BEST POSE = 5.0 6.0 0.1 BEST RESPONSE = 0.9",
    "G02_LOOP event=check scan=3 time=4.000000000 info=COARSE RESPONSE: .9 (> .35)",
    "BEST POSE = 5.1 6.1 0.2 BEST RESPONSE = 0.2",
    "G02_LOOP event=check scan=3 time=4.000000000 info=FINE RESPONSE: .2 (> .45)",
    "G02_LOOP event=check scan=3 time=4.000000000 info=REJECTED!",
    "G02_LOOP event=scan_end scan=3 time=4.000000000 info=no_accepted_closure",
]
for line in lines:
    events.feed(line)
assert len(events.events)==3
assert [e["accepted"] for e in events.events]==[True,False,False]
assert [e["reason"] for e in events.events][1:]==["coarse threshold","fine threshold"]
assert events.events[0]["candidate_map_pose"]==[1.1,2.1,.2]
assert events.events[2]["candidate_map_pose"]==[5.1,6.1,.2]
assert [e["time"] for e in events.events]==[2.,3.,4.]
print("LOOP_EVENT_CONTROLS_PASS accepted=1 coarse_rejected=1 fine_rejected=1")
