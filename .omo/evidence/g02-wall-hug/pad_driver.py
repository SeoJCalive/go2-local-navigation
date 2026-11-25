"""Run only qpos feedback and FIFO pad control on a 20 ms absolute schedule.

The parent passes its read-only simulator-memory descriptor and the established
FIFO descriptor. No ROS callbacks, screenshots, or Karto log parsing run here.
The existing G01 truth/event/neutral definitions retain their validated ABI and
pad packet semantics. The parent stays the process owner and can stop this
child before reclaiming the FIFO. Every exit neutralizes the pad.
"""
import ast
import json
import math
import os
from pathlib import Path
import select
import signal
import struct
import sys
import time
import traceback
import numpy as np
from path_tracker import PathTracker

ROOT=Path(__file__).resolve().parent
PERIOD=.020


def main(spec_path):
    spec=json.loads(Path(spec_path).read_text())
    output=Path(spec["output_directory"])
    namespace=dict(spec,os=os,struct=struct,math=math,time=time)
    tree=ast.parse((ROOT.parent/"g01-owner-gui/owner_gui.py").read_text())
    definitions=[n for n in tree.body if isinstance(n,ast.FunctionDef)
                 and n.name in ("truth","event","neutral")]
    assert len(definitions)==3
    exec(compile(ast.Module(body=definitions,type_ignores=[]),
                 "established_fifo_and_truth","exec"),namespace)
    tracker=PathTracker(json.loads(Path(spec["route_path"]).read_text())["waypoints"])
    stopping=False
    def stop(signum,frame):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    state_path=output/"driver-status.json"
    temporary=output/".driver-status.json.tmp"
    result={"period_s":PERIOD,"finished":False,"errors":[]}
    rows=[]
    gaps=[]
    sim_gaps=[]
    crosses=[]
    last_tick=None
    last_sim=None
    last_status=-math.inf
    last_index=-1
    best=0.
    progress_at=time.monotonic()
    deadline=time.monotonic()
    record=None
    previous_costs={}
    try:
        with (output/"driver-trace.jsonl").open("w") as trace:
            while not stopping:
                remaining=deadline-time.monotonic()
                if remaining>0:
                    select.select([],[],[],remaining)
                if stopping:
                    break
                tick=time.monotonic()
                p=namespace["truth"]()
                truth_done=time.monotonic()
                command,tracking=tracker.command(p)
                tracking_done=time.monotonic()
                costs={"truth":truth_done-tick,"tracking":tracking_done-truth_done}
                gap=0. if last_tick is None else tick-last_tick
                sim_gap=0. if last_sim is None else p["sim_time"]-last_sim
                if last_tick is not None:
                    gaps.append(gap)
                    sim_gaps.append(sim_gap)
                last_tick,last_sim=tick,p["sim_time"]
                record={"time":p["sim_time"],"truth":p,**tracking,
                    "command":command,"driver_wall_monotonic_s":tick,
                    "driver_loop_gap_wall_s":gap,"driver_loop_gap_sim_s":sim_gap,
                    "deadline_lateness_s":max(0.,tick-deadline),
                    "previous_iteration_cost_s":previous_costs}
                trace_started=time.monotonic()
                trace.write(json.dumps(record)+"\n")
                trace.flush()
                costs["trace_write"]=time.monotonic()-trace_started
                crosses.append(abs(tracking["cross_track_m"]))
                if tracking["index"]!=last_index:
                    last_index=tracking["index"]
                    rows.append({"index":last_index,"time":p["sim_time"],"truth":p,
                        "target":tracker.waypoints[last_index]})
                if tracking["done"]:
                    result["finished"]=True
                    break
                if tracking["s_m"]>best+.04:
                    best=tracking["s_m"]
                    progress_at=tick
                if tick-progress_at>25:
                    raise RuntimeError(f"no 4 cm path progress in 25 s: {tracking}; {p}")
                q=p["qpos"]
                roll=math.atan2(2*(q[3]*q[4]+q[5]*q[6]),1-2*(q[4]**2+q[5]**2))
                pitch=math.asin(max(-1,min(1,2*(q[3]*q[5]-q[6]*q[4]))))
                if p["z"]<.18 or max(abs(roll),abs(pitch))>math.pi/3:
                    raise RuntimeError("fall detected "+str(p))
                vx,vy,wz=command
                pad_started=time.monotonic()
                namespace["event"](round(-vx/(4 if vx>=0 else 2)*32767),2,1)
                namespace["event"](round(-vy*32767),2,0)
                namespace["event"](round(-wz/2*32767),2,3)
                costs["pad_write"]=time.monotonic()-pad_started
                status_started=time.monotonic()
                if tick-last_status>=.1:
                    temporary.write_text(json.dumps(record))
                    os.replace(temporary,state_path)
                    last_status=tick
                costs["status_write"]=time.monotonic()-status_started
                costs["iteration_total"]=time.monotonic()-tick
                previous_costs=costs
                deadline+=PERIOD
    except Exception:
        result["errors"].append(traceback.format_exc())
    finally:
        namespace["neutral"]()
        result.update(last=record,interrupted=stopping,mapping_trace=rows,
            max_driver_loop_gap_wall_s=max(gaps,default=0.),
            p95_driver_loop_gap_wall_s=float(np.percentile(gaps,95)) if gaps else 0.,
            max_driver_loop_gap_sim_s=max(sim_gaps,default=0.),
            cross_track_error_m={"max":max(crosses,default=0.),
                "p95":float(np.percentile(crosses,95)) if crosses else 0.})
        temporary.write_text(json.dumps(record))
        os.replace(temporary,state_path)
        (output/"driver-result.json").write_text(json.dumps(result,indent=2))
    print("PAD_DRIVER_FINISHED "+json.dumps({
        key:result[key] for key in ("finished","errors","max_driver_loop_gap_wall_s",
                                   "max_driver_loop_gap_sim_s")}),flush=True)
    return 0 if result["finished"] and not result["errors"] else 1


if __name__=="__main__":
    sys.exit(main(sys.argv[1]))
