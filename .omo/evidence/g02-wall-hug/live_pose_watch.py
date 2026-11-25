"""Read-only live SLAM/GT monitor; never opens a pad, ROS node, or simulator.

Complete telemetry lines are consumed incrementally. The first 0.3 m crossing
includes its actual preceding loop edge, GT node positions, raw scan snapshots,
and measured driver gaps. Run under a monitor; exit when the driver finishes.
"""
import bisect
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import select
import signal
import time

ROOT=Path(__file__).resolve().parent
run=json.loads((ROOT/"result.json").read_text())
attempt=run["attempt"]
yaw,tx,ty=run["saved_map_world_transform"]
c,s=math.cos(yaw),math.sin(yaw)
stopping=False
def stop(*_):
    global stopping
    stopping=True
signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,stop)

class Lines:
    def __init__(self,name):
        self.file=(ROOT/name).open()
        self.suffix=""
    def read(self):
        while True:
            new=self.file.readline()
            if not new:
                return
            line=self.suffix+new
            if not line.endswith("\n"):
                self.suffix=line
                return
            self.suffix=""
            yield line

truth=Lines("physics-truth.csv")
poses=Lines("slam-pose.csv")
edge_lines=Lines("karto-edges.jsonl")
driver=Lines("driver-trace.jsonl")
gt_t=[]; gt_x=[]; gt_y=[]
edges=[]; pending=[]
driver_gaps=[]
max_error=0.
first=None
last_report=-math.inf
def gt_at(t):
    i=bisect.bisect_right(gt_t,t)-1
    if i<0 or t>gt_t[-1]:
        raise ValueError(f"GT does not cover {t}")
    if i==len(gt_t)-1:
        return [gt_x[i],gt_y[i]]
    alpha=(t-gt_t[i])/(gt_t[i+1]-gt_t[i])
    return [gt_x[i]+alpha*(gt_x[i+1]-gt_x[i]),
            gt_y[i]+alpha*(gt_y[i+1]-gt_y[i])]

def progress(step,message):
    images=[p for p in (ROOT/"live").glob("[0-9][0-9][0-9][0-9][0-9][0-9]_*.png")]
    image=max(images,key=lambda p:p.stat().st_mtime_ns).name
    line=f"{datetime.now():%H:%M:%S} | {step} | {message} | {image}\n"
    with (ROOT/"live/progress.log").open("a") as out:
        out.write(line)
    print(line.strip(),flush=True)

def alert(t,error,slam,expected):
    preceding=[e for e in edges if e["loop"] and e["to_time"]<=t+1e-6]
    pair=max(preceding,key=lambda e:e["to_time"]) if preceding else None
    record={"attempt":attempt,"sim_time":t,"error_m":error,"slam_xy":slam,
            "gt_map_xy":expected,"last_accepted_loop_edge":pair,
            "native_raw_scan_file":"karto-scans.jsonl",
            "native_edge_file":"karto-edges.jsonl","raw_scan_files":[],
            "driver_max_gap_wall_s_before_crossing":max(
                (gap for stamp,gap in driver_gaps if stamp<=t),default=0.)}
    if pair:
        a=gt_at(pair["from_time"]); b=gt_at(pair["to_time"])
        record["edge_gt_world_xy"]={"from":a,"to":b}
        record["edge_gt_separation_m"]=math.dist(a,b)
        wanted={pair["from_scan_id"],pair["to_scan_id"]}
        with (ROOT/"karto-scans.jsonl").open() as raw:
            for line in raw:
                if not line.endswith("\n"):
                    break
                scan=json.loads(line)
                if scan["scan_id"] in wanted:
                    path=ROOT/"live"/f"attempt{attempt}-scan-{scan['scan_id']}.json"
                    path.write_text(json.dumps(scan,indent=2))
                    record["raw_scan_files"].append(str(path.relative_to(ROOT)))
                    wanted.remove(scan["scan_id"])
                    if not wanted:
                        break
        record["missing_raw_scan_ids"]=sorted(wanted)
    (ROOT/"live"/f"attempt{attempt}-first-error-over-0.3m.json").write_text(
        json.dumps(record,indent=2))
    pair_text=f"{pair['from_scan_id']}->{pair['to_scan_id']} at {pair['to_time']:.3f}s" if pair else "없음"
    progress("G02/pose-alert",
        f"누적 기록에서 최초 SLAM-qpos 오차 0.3m 초과 확인: sim {t:.3f}s, "
        f"오차 {error:.6f}m, 마지막 수락 edge {pair_text}, "
        f"raw {','.join(record['raw_scan_files'])}; "
        f"초과 전 driver 최대 간격 {record['driver_max_gap_wall_s_before_crossing']:.6f}s. "
        "접촉·stall·fall 중단 기준 외에는 계속 주행한다")
    return record

deadline=time.monotonic()+3600
with (ROOT/"live"/f"attempt{attempt}-slam-error-live.csv").open("w") as out:
    writer=csv.writer(out)
    writer.writerow(["time","error_m","slam_x","slam_y","gt_map_x","gt_map_y"])
    while not stopping:
        for line in truth.read():
            if line.startswith("time,"):
                continue
            values=line.strip().split(",")
            t,x,y=map(float,values[:3])
            if gt_t and t==gt_t[-1]:
                gt_x[-1]=x; gt_y[-1]=y
            else:
                if gt_t and t<gt_t[-1]:
                    raise ValueError("GT clock decreased")
                gt_t.append(t); gt_x.append(x); gt_y.append(y)
        edges.extend(json.loads(line) for line in edge_lines.read())
        for line in driver.read():
            d=json.loads(line)
            driver_gaps.append((d["time"],d["driver_loop_gap_wall_s"]))
        for line in poses.read():
            if not line.startswith("time,"):
                pending.append(list(map(float,line.strip().split(","))))
        processed=0
        for t,x,y,_ in pending:
            if not gt_t or t>gt_t[-1]:
                break
            gx,gy=gt_at(t)
            expected=[c*gx-s*gy+tx,s*gx+c*gy+ty]
            error=math.dist([x,y],expected)
            writer.writerow([t,error,x,y,*expected])
            max_error=max(max_error,error)
            if error>.3 and first is None:
                first=alert(t,error,[x,y],expected)
            processed+=1
        pending=pending[processed:]
        out.flush()
        status={"attempt":attempt,"latest_sim_time":gt_t[-1] if gt_t else None,
            "max_pose_error_m":max_error,"first_error_over_0_3m":first,
            "driver_max_gap_wall_s":max((g for _,g in driver_gaps),default=0.)}
        temporary=ROOT/"live"/f".pose-status-{os.getpid()}.json"
        temporary.write_text(json.dumps(status,indent=2))
        os.replace(temporary,ROOT/"live/slam-error-status.json")
        if gt_t and gt_t[-1]-last_report>=60:
            progress("G02/pose-monitor",
                f"sim {gt_t[-1]:.3f}s, 누적 pose 최대 오차 {max_error:.6f}m, "
                f"driver 최대 간격 {status['driver_max_gap_wall_s']:.6f}s; 별도 읽기 전용 감시 중")
            last_report=gt_t[-1]
        if (ROOT/"driver-result.json").exists():
            print("POSE_MONITOR_FINISHED "+json.dumps(status),flush=True)
            break
        if time.monotonic()>deadline:
            raise TimeoutError("bounded live pose monitor")
        select.select([],[],[],1.)
