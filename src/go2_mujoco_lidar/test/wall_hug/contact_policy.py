"""Maintained G02 integer microsecond contact and motion abort classifiers; CSV adapter to violation and metrics; promoted from commit 123f8b7 evidence.

Inputs and outputs follow the functions below; see README.md in this directory
and docs/stabilization-2026-11/G02_WALL_HUG.md for provenance and limitations.
Simulation/offline diagnostics only; real robot safety is unconfirmed.
"""
import math
from decimal import Decimal



class ContactPolicy:
    def __init__(self):
        self.metrics={
            "mapping_nonfoot_contacts":0,"mapping_contact_episodes":0,
            "mapping_max_contact_depth_m":0.,"mapping_max_contact_duration_s":0.,
            "mapping_max_deep_contact_duration_s":0.}
        self.states={}
        self.frame_stamp=None
        self.frame={}
        self.violation=None

    def feed(self,rows,complete_before_s):
        for fields in rows:
            stamp=int(Decimal(str(fields[0]))*1_000_000)
            if self.frame_stamp is not None and stamp!=self.frame_stamp:
                assert stamp>self.frame_stamp
                self.finish_frame()
            self.frame_stamp=stamp
            key=(fields[1],fields[4])
            depth=-float(fields[3])
            self.frame[key]=max(depth,self.frame.get(key,0.))
            self.metrics["mapping_nonfoot_contacts"]+=1
            self.metrics["mapping_max_contact_depth_m"]=max(
                self.metrics["mapping_max_contact_depth_m"],depth)
        if self.frame_stamp is not None and (complete_before_s==float("inf") or
                self.frame_stamp<int(Decimal(str(complete_before_s))*1_000_000)):
            self.finish_frame()
        return self.violation

    def finish_frame(self):
        stamp=self.frame_stamp
        for key,depth in self.frame.items():
            previous=self.states.get(key)
            continuous=previous is not None and stamp-previous["last"]<=3000
            first=previous["first"] if continuous else stamp
            if not continuous:
                self.metrics["mapping_contact_episodes"]+=1
            deep_first=None
            if depth>.005:
                deep_first=(previous["deep_first"] if continuous and
                    previous["deep_first"] is not None else stamp)
            duration=stamp-first
            deep_duration=stamp-deep_first if deep_first is not None else 0
            self.states[key]={"first":first,"last":stamp,"deep_first":deep_first}
            self.metrics["mapping_max_contact_duration_s"]=max(
                self.metrics["mapping_max_contact_duration_s"],duration/1_000_000)
            self.metrics["mapping_max_deep_contact_duration_s"]=max(
                self.metrics["mapping_max_deep_contact_duration_s"],deep_duration/1_000_000)
            if (depth>.015 or deep_duration>50000 or duration>1_000_000) and self.violation is None:
                self.violation=(f"contact policy: time={stamp/1_000_000:.6f} "
                    f"world/body={key} depth={depth:.9f}m "
                    f"above5mm={deep_duration/1_000_000:.6f}s "
                    f"any_contact={duration/1_000_000:.6f}s")
        self.frame.clear()
        self.frame_stamp=None


class MotionPolicy:
    """Classify 25 s / 4 cm stall and qpos fall on integer wall microseconds."""
    def __init__(self, stamp_us):
        self.best = 0.
        self.progress_at = stamp_us

    def check(self, stamp_us, arc_m, pose):
        if arc_m > self.best + .04:
            self.best, self.progress_at = arc_m, stamp_us
        if stamp_us - self.progress_at > 25_000_000:
            return "stall: no 4 cm path progress in 25 s"
        q = pose["qpos"]
        roll = math.atan2(2*(q[3]*q[4]+q[5]*q[6]),1-2*(q[4]**2+q[5]**2))
        pitch = math.asin(max(-1,min(1,2*(q[3]*q[5]-q[6]*q[4]))))
        if pose["z"] < .18 or max(abs(roll), abs(pitch)) > math.pi/3:
            return "fall detected"
        return None
