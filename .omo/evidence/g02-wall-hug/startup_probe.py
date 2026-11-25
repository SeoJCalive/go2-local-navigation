"""Isolate GUI DDS startup with/without mj_step observation; never start RL."""
import ast
from pathlib import Path
import json
import os
import shutil
import sys
import time

ROOT=Path(__file__).resolve().parent
tree=ast.parse((ROOT/"gui_run.py").read_text())
body=[]
for statement in tree.body:
    if isinstance(statement,ast.Assign) and any(
            isinstance(t,ast.Name) and t.id=="began" for t in statement.targets):
        break
    body.append(statement)
ns={"__file__":str(ROOT/"gui_run.py"),"__name__":"probe"}
exec(compile(ast.Module(body=body,type_ignores=[]),"gui_run.py","exec"),ns)
g=ns["g"]
instrumented="--instrumented" in sys.argv
output=ROOT/("probe-instrumented" if instrumented else "probe-plain")
output.mkdir(exist_ok=True)
(output/"live").mkdir(exist_ok=True)
for name in ("offsets","live_capture.py","config-mapping.yaml","config-nav.yaml"):
    shutil.copy2(ROOT/name,output/name)
for name in ("config-mapping.yaml","config-nav.yaml"):
    p=output/name
    p.write_text(p.read_text().replace(str(ROOT),str(output)))
g["ROOT"],g["FIFO"]=output,output/"owner-pad.fifo"
g["RESULT"]={"goals":[],"steps":{},"shutdowns":[],"errors":[],"screenshots":[]}
try:
    g["prepare"]()
    os.mkfifo(g["FIFO"]) if not g["FIFO"].exists() else None
    g["pad_fd"]=os.open(g["FIFO"],os.O_RDWR|os.O_NONBLOCK)
    stage=output/"simulate"
    (stage/"build").mkdir(parents=True,exist_ok=True)
    binary=stage/"build/unitree_mujoco"
    if not binary.exists():
        os.link(g["SIMROOT"]/"build/unitree_mujoco",binary)
    shutil.copy2(output/"config-mapping.yaml",stage/"config.yaml")
    extra=({"LD_PRELOAD":str(ROOT/"physics_observer.so"),
            "G02_PHYSICS_LOG":str(output/"physics")} if instrumented else {})
    sim=g["start"]("simulator",[str(binary),"-r","go2","-s",str(g["WORLD"]),
                    "-i","41","-n","lo"],native=True,env_extra=extra)
    began=time.monotonic()
    try:
        g["wait"](lambda:len(g["samples"]["sport"])>10 or sim.poll() is not None,
                  30,"probe DDS")
        ready=len(g["samples"]["sport"])>10
    except TimeoutError:
        ready=False
    result={"instrumented":instrumented,"dds_ready":ready,
            "sport_count":len(g["samples"]["sport"]),"wall_sec":time.monotonic()-began,
            "sim_returncode":sim.poll()}
    g["arrange"]()
    def rendered_scene():
        shot=g["capture"]("probe-render")
        pixels=ns["np"].asarray(ns["Image"].open(output/shot))
        return int((pixels[60:650,190:620,:3].mean(axis=2)>35).sum())>30000
    g["wait"](rendered_scene,15,"tracking camera rendered scene")
    g["capture"]("probe-tracking",True)
    print("STARTUP_PROBE "+json.dumps(result),flush=True)
    (output/"probe-result.json").write_text(json.dumps(result,indent=2))
finally:
    for process in reversed(g["owned"].processes):
        g["stop"](process)
    for name in ("pad_fd","mem_fd"):
        if g[name] is not None:
            os.close(g[name])
    g["FIFO"].unlink(missing_ok=True)
    ns["node"].destroy_node()
    ns["rclpy"].shutdown()
