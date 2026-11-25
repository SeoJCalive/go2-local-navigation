"""Maintained G02 independent 20 ms driver; inherited read-only memory and pad descriptors to trace/status JSON; promoted from commit 123f8b7 evidence.

Inputs and outputs follow the functions below; see README.md in this directory
and docs/stabilization-2026-11/G02_WALL_HUG.md for provenance and limitations.
Simulation/offline diagnostics only; real robot safety is unconfirmed.
"""
"""Run only qpos feedback and FIFO pad control on a 20 ms absolute schedule.

The parent passes its read-only simulator-memory descriptor and the established
FIFO descriptor. No ROS callbacks, screenshots, or Karto log parsing run here.
The existing G01 truth/event/neutral definitions retain their validated ABI and
pad packet semantics. The parent stays the process owner and can stop this
child before reclaiming the FIFO. Every exit neutralizes the pad.
"""
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
from contact_policy import MotionPolicy

PERIOD=.020


def main(spec_path):
    spec=json.loads(Path(spec_path).read_text())
    output=Path(spec["output_directory"])
    def event(value, kind, number):
        os.write(pad_fd, struct.pack("IhBB", int(time.monotonic()*1000) % 2**32,
                                     int(value), kind, number))


    def neutral():
        for a in range(8):
            event(0, 2, a)
        for b in range(12):
            event(0, 1, b)

    def truth():
        ptr = struct.unpack("Q", os.pread(mem_fd, 8, d_address))[0]
        qp = struct.unpack("Q", os.pread(mem_fd, 8, ptr + qpos_offset))[0]
        vals = struct.unpack("7d", os.pread(mem_fd, 56, qp))
        w, x, y, z = vals[3:]
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        t = struct.unpack("d", os.pread(mem_fd, 8, ptr+time_offset))[0]
        return {"x": vals[0], "y": vals[1], "z": vals[2], "yaw": yaw,
                "qpos": vals, "sim_time": t}
    mem_fd, pad_fd = spec["mem_fd"], spec["pad_fd"]
    d_address, qpos_offset, time_offset = (spec[k] for k in ("d_address", "qpos_offset", "time_offset"))
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
    motion=MotionPolicy(round(time.monotonic()*1_000_000))
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
                p=truth()
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
                violation=motion.check(round(tick*1_000_000),tracking["s_m"],p)
                if violation:
                    raise RuntimeError(violation)
                vx,vy,wz=command
                pad_started=time.monotonic()
                event(round(-vx/(4 if vx>=0 else 2)*32767),2,1)
                event(round(-vy*32767),2,0)
                event(round(-wz/2*32767),2,3)
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
        neutral()
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
