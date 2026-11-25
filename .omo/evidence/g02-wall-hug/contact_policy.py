"""Classify every non-foot contact using physics timestamps, never read timing.

Aggregate contact points by world geometry/body and physics frame before testing
depth. A shallow point cannot reset another point's deep contact in that frame.
Abort above 15 mm once, above 5 mm for more than 50 ms, or any contact over 1 s.
The original per-point CSV remains the complete tap record.
"""


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
            stamp=round(float(fields[0])*1_000_000)
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
                self.frame_stamp<round(complete_before_s*1_000_000)):
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
