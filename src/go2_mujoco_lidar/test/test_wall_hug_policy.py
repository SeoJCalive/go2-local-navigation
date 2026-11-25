"""G02 behavioral regressions: numerical controls and OS signals, no robot or GUI.

Inputs are independent synthetic poses/contacts/maps; pytest reports failures.
See wall_hug/README.md and G02_WALL_HUG.md for thresholds and provenance.
These checks establish harness contracts; physical validity is unconfirmed.
"""
from pathlib import Path
import sys
import pytest
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/"wall_hug"))
from contact_policy import ContactPolicy, MotionPolicy
from path_tracker import PathTracker
import math


def contact(samples, chunks=False):
    policy = ContactPolicy()
    for us, depths in samples:
        stamp = f"{us//1_000_000}.{us%1_000_000:06d}"
        rows = [[stamp,"wall","geom",str(-depth),"FR_calf","17"] for depth in depths]
        if chunks:
            for row in rows:
                policy.feed([row], float(stamp))
        else:
            policy.feed(rows, float(stamp))
    policy.feed([], float("inf"))
    return policy


@pytest.mark.parametrize("depth,aborted",[(.000022,False),(.005,False),(.005001,False),(.015,False),(.015001,True)])
def test_instant_depth(depth, aborted):
    assert bool(contact([(0,[depth])]).violation) == aborted


@pytest.mark.parametrize("end,depth,aborted",[(50000,.006,False),(50001,.006,True),(1_000_000,.001,False),(1_000_001,.001,True),(6000,.005925174,False)])
def test_duration_boundary_microseconds(end, depth, aborted):
    stamps = list(range(0,end,2000)) + [end]
    result = contact([(us,[depth]) for us in stamps])
    assert bool(result.violation) == aborted
    assert result.metrics["mapping_max_contact_duration_s"] == end/1_000_000


def test_frame_aggregation_chunk_watermark_and_shallow_reset():
    samples = [(k*2000,[.006,.001]) for k in range(27)]
    one, chunked = contact(samples), contact(samples, True)
    assert one.violation and chunked.violation
    assert one.metrics == chunked.metrics
    samples[13] = (26000,[.001])
    assert not contact(samples, True).violation
    assert contact([(0,[.001]),(4000,[.001])]).metrics["mapping_contact_episodes"] == 2


def pose(z=.33, angle=0):
    return {"z":z,"qpos":[0,0,z,math.cos(angle/2),math.sin(angle/2),0,0]}


def test_stall_and_progress_boundary():
    policy = MotionPolicy(0)
    assert policy.check(25_000_000,.04,pose()) is None
    assert policy.check(25_000_001,.04,pose()).startswith("stall")
    policy = MotionPolicy(0)
    assert policy.check(24_000_000,.040001,pose()) is None
    assert policy.check(49_000_000,.05,pose()) is None
    assert policy.check(49_000_001,.05,pose())


@pytest.mark.parametrize("p,falls",[(pose(.18),False),(pose(.179999),True),(pose(angle=1.04),False),(pose(angle=1.06),True)])
def test_fall(p, falls):
    assert bool(MotionPolicy(0).check(0,0,p)) == falls


def tracker(points):
    return PathTracker([{"xy":p} for p in points])


def test_continuous_lookahead_and_cross_track():
    t = tracker([[0,0],[0,0],[1,0],[2,0]])
    command, status = t.command({"x":.1,"y":.05,"yaw":0})
    assert command[0] == pytest.approx(.30)
    assert command[1] < 0 and command[2] < 0
    assert status["cross_track_m"] == pytest.approx(.05)
    assert status["target"] == pytest.approx([.32,0])
    assert len(t.waypoints) == 3
    _, backwards = t.command({"x":.05,"y":0,"yaw":0})
    assert backwards["s_m"] == pytest.approx(.1)


def test_corner_turn_in_place_and_completion():
    t = tracker([[0,0],[.2,0],[.2,1]])
    command, status = t.command({"x":0,"y":0,"yaw":0})
    assert status["corner"] and command[0] == pytest.approx(.24)
    command, _ = t.command({"x":0,"y":0,"yaw":math.pi})
    assert command[:2] == [0,0] and abs(command[2]) <= .65
    t = tracker([[0,0],[.1,0],[.2,0]])
    _, status = t.command({"x":.2,"y":0,"yaw":0})
    assert status["done"]


def test_driver_absolute_schedule_does_not_accumulate_work_time(tmp_path, monkeypatch):
    """Run actual main with a synthetic clock; exact ticks replace wall-time limits."""
    import json
    import struct
    import pad_driver
    clock={"now":0.,"samples":0}
    waits=[]
    packets=[]
    def wait(readers,writers,errors,remaining):
        assert readers==writers==errors==[]
        waits.append(remaining)
        clock["now"]+=remaining
        return [],[],[]
    def pread(fd,size,address):
        if address==0:
            return struct.pack("Q",64)
        if address==72:
            return struct.pack("Q",128)
        if address==128:
            x=clock["samples"]*.01
            clock["samples"]+=1
            clock["now"]+=.003  # Known work must be subtracted from next wait.
            return struct.pack("7d",x,0,.33,1,0,0,0)
        assert address==80
        return struct.pack("d",clock["samples"]*.02)
    monkeypatch.setattr(pad_driver.time,"monotonic",lambda:clock["now"])
    monkeypatch.setattr(pad_driver.select,"select",wait)
    monkeypatch.setattr(pad_driver.os,"pread",pread)
    monkeypatch.setattr(pad_driver.os,"write",lambda fd,p:packets.append(struct.unpack("IhBB",p)))
    monkeypatch.setattr(pad_driver.signal,"signal",lambda *_:None)
    route=tmp_path/"route.json"
    route.write_text(json.dumps({"waypoints":[{"xy":[x,0]} for x in (0,.1,.2)]}))
    spec=tmp_path/"spec.json"
    spec.write_text(json.dumps(dict(mem_fd=8,pad_fd=7,d_address=0,qpos_offset=8,
        time_offset=16,route_path=str(route),output_directory=str(tmp_path))))
    assert pad_driver.main(spec)==0
    trace=[json.loads(line) for line in (tmp_path/"driver-trace.jsonl").read_text().splitlines()]
    assert len(trace)>5
    assert [r["driver_wall_monotonic_s"] for r in trace] == pytest.approx(
        [k*.02 for k in range(len(trace))])
    assert waits == pytest.approx([.017]*len(waits))
    result=json.loads((tmp_path/"driver-result.json").read_text())
    assert result["max_driver_loop_gap_wall_s"] == pytest.approx(.02)
    assert [(v,k,n) for _,v,k,n in packets[-20:]] == (
        [(0,2,k) for k in range(8)]+[(0,1,k) for k in range(12)])
