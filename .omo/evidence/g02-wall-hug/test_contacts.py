"""Independent boundary fixtures for the exact policy used by the GUI run."""
from contact_policy import ContactPolicy

def check(samples):
    rows=[[str(t),"wall","unnamed",str(-depth),"FR_calf","17"] for t,depth in samples]
    policy=ContactPolicy()
    return policy.metrics,bool(policy.feed(rows,float("inf")))

r,aborted=check([(0,.000022),(.002,.000022)])
assert not aborted and r["mapping_nonfoot_contacts"]==2 and r["mapping_contact_episodes"]==1
assert r["mapping_max_contact_depth_m"]==.000022
assert not check([(0,.005)])[1]
assert not check([(0,.005001)])[1]
assert not check([(0,.015)])[1]
assert check([(0,.015001)])[1]
assert not check([(k*.002,.005925174) for k in range(4)])[1]
assert not check([(k*.002,.005001) for k in range(26)])[1]
assert check([(k*.002,.005001) for k in range(27)])[1]
assert not check([(k*.002,.000022) for k in range(501)])[1]
assert check([(k*.002,.000022) for k in range(502)])[1]
r,aborted=check([(0,.000022),(.004,.000022)])
assert not aborted and r["mapping_contact_episodes"]==2
# A shallow and deep point on the same frame still sustain the deep contact.
same_frame=[(k*.002,depth) for k in range(27) for depth in (.006,.001)]
assert check(same_frame)[1]
# An actual shallow frame, unlike a second point, interrupts deep duration.
assert not check([(k*.002,.006 if k!=13 else .001) for k in range(27)])[1]
# Reading one frame across chunks gives the same result.
policy=ContactPolicy()
for k in range(27):
    policy.feed([[str(k*.002),"wall","unnamed","-.006","FR_calf","17"]],k*.002)
    policy.feed([[str(k*.002),"wall","unnamed","-.001","FR_calf","17"]],k*.002)
assert policy.feed([],float("inf"))
print("CONTACT_POLICY_PASS graze22um=continue tap5.925mm6ms=continue "
      "15mm=continue above15mm=abort above5mm50ms=continue above5mm52ms=abort "
      "duration1s=continue above1s=abort gap4ms=new_episode same_frame=max_depth")
