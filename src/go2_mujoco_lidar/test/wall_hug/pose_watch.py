"""Read-only SLAM/qpos monitor promoted from live_pose_watch.py (123f8b7).

prepare_run truncates streams BEFORE native writers/watchers start (attempt-7 stale
reader fix). Lines holds partial frames. PoseMonitor interpolates bracketed GT,
transforms world to map and associates only new inserted loops. The CLI replays
a prepared directory to JSON; no pad, ROS or simulator is opened. See README.md
and G02_WALL_HUG.md; this is telemetry evidence, not a SLAM correctness proof.
"""
import bisect
import json
import math
from pathlib import Path

STREAMS=("physics-truth.csv","slam-pose.csv","karto-edges.jsonl","driver-trace.jsonl","karto-scans.jsonl","karto-candidates.jsonl")


def prepare_run(root):
    """Call before ANY producer or reader; previous evidence must be archived first."""
    root=Path(root)
    for name in STREAMS:
        (root/name).write_text("")


class Lines:
    """Consume complete newline records only, preserving split JSON/CSV suffixes."""
    def __init__(self,path):
        self.file=Path(path).open()
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

    def close(self):
        self.file.close()


class PoseMonitor:
    def __init__(self, transform):
        yaw,self.tx,self.ty=transform
        self.c,self.s=math.cos(yaw),math.sin(yaw)
        self.gt_t=[]; self.gt_x=[]; self.gt_y=[]
        self.edges=[]; self.pending=[]; self.driver_gaps=[]
        self.max_error=0.
        self.first=None

    def gt_at(self,t):
        i=bisect.bisect_right(self.gt_t,t)-1
        if i<0 or t>self.gt_t[-1]:
            raise ValueError(f"GT does not cover {t}")
        if i==len(self.gt_t)-1:
            return [self.gt_x[i],self.gt_y[i]]
        alpha=(t-self.gt_t[i])/(self.gt_t[i+1]-self.gt_t[i])
        return [self.gt_x[i]+alpha*(self.gt_x[i+1]-self.gt_x[i]),
                self.gt_y[i]+alpha*(self.gt_y[i+1]-self.gt_y[i])]

    def feed(self, truth=(), poses=(), edges=(), driver=()):
        for t,x,y in truth:
            if self.gt_t and t<self.gt_t[-1]:
                raise ValueError("GT clock decreased")
            if self.gt_t and t==self.gt_t[-1]:
                self.gt_x[-1],self.gt_y[-1]=x,y
            else:
                self.gt_t.append(t); self.gt_x.append(x); self.gt_y.append(y)
        self.edges.extend(e for e in edges if e["loop"] and e["new_edge"])
        self.driver_gaps.extend((r["time"],r["driver_loop_gap_wall_s"]) for r in driver)
        self.pending.extend(poses)
        processed=0
        measurements=[]
        for t,x,y in self.pending:
            if not self.gt_t or t>self.gt_t[-1]:
                break
            gx,gy=self.gt_at(t)
            expected=[self.c*gx-self.s*gy+self.tx,self.s*gx+self.c*gy+self.ty]
            error=math.dist([x,y],expected)
            measurements.append({"time":t,"error_m":error,"gt_map_xy":expected})
            self.max_error=max(self.max_error,error)
            if error>.3 and self.first is None:
                preceding=[e for e in self.edges if e["to_time"]<=t+1e-6]
                self.first={"sim_time":t,"error_m":error,"slam_xy":[x,y],
                    "gt_map_xy":expected,
                    "last_accepted_loop_edge":max(preceding,key=lambda e:e["to_time"]) if preceding else None,
                    "driver_max_gap_wall_s_before_crossing":max((g for stamp,g in self.driver_gaps if stamp<=t),default=0.)}
            processed+=1
        self.pending=self.pending[processed:]
        return measurements


def replay(root, transform):
    import csv
    root=Path(root)
    def records(name):
        with (root/name).open() as stream:
            return [json.loads(line) for line in stream if line.endswith("\n")]
    with (root/"physics-truth.csv").open() as stream:
        truth=[tuple(float(r[k]) for k in ("time","x","y")) for r in csv.DictReader(stream)]
    with (root/"slam-pose.csv").open() as stream:
        poses=[tuple(float(r[k]) for k in ("time","x","y")) for r in csv.DictReader(stream)]
    monitor=PoseMonitor(transform)
    monitor.feed(truth,poses,records("karto-edges.jsonl"),records("driver-trace.jsonl"))
    if monitor.pending:
        raise ValueError("unbracketed pose samples remain")
    return {"max_pose_error_m":monitor.max_error,"first_error_over_0_3m":monitor.first}


if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory",type=Path)
    parser.add_argument("--transform",nargs=3,type=float,required=True)
    args=parser.parse_args()
    print(json.dumps(replay(args.directory,args.transform),indent=2))
