"""G02 behavioral regressions: numerical controls and OS signals, no robot or GUI.

Inputs are independent synthetic poses/contacts/maps; pytest reports failures.
See wall_hug/README.md and G02_WALL_HUG.md for thresholds and provenance.
These checks establish harness contracts; physical validity is unconfirmed.
Native CSV shutdown suffixes use the production complete-line framer.
"""
from pathlib import Path
import sys
import pytest
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/"wall_hug"))
import json
import math
from pose_watch import Lines, PoseMonitor, prepare_run, replay as replay_pose


def test_stale_previous_attempt_is_cleared_before_reader(tmp_path):
    path=tmp_path/"karto-edges.jsonl"
    stale={"loop":True,"new_edge":True,"from_scan_id":140,"to_scan_id":275,"to_time":201.302}
    path.write_text(json.dumps(stale)+"\n")
    prepare_run(tmp_path)
    reader=Lines(path)
    try:
        assert list(reader.read()) == []
        fresh=dict(stale,from_scan_id=148,to_scan_id=284,to_time=202.344)
        with path.open("a") as stream:
            stream.write(json.dumps(fresh)[:30])
        assert list(reader.read()) == []
        with path.open("a") as stream:
            stream.write(json.dumps(fresh)[30:]+"\n")
        assert [json.loads(line) for line in reader.read()] == [fresh]
        monitor=PoseMonitor([0,0,0])
        monitor.feed([(202,0,0),(203,0,0)],[(202.4,1,0)],edges=[fresh])
        assert monitor.first["last_accepted_loop_edge"]["from_scan_id"] == 148
    finally:
        reader.close()


def test_gt_bracket_transform_and_real_new_edges():
    monitor=PoseMonitor([math.pi/2,2,3])
    assert monitor.feed([(0,0,0)],[(.5,2,3.5)]) == []
    samples=monitor.feed([(1,1,0)])
    assert samples[0]["error_m"] == pytest.approx(0)
    edges=[{"loop":True,"new_edge":True,"to_time":1,"from_scan_id":1},
           {"loop":True,"new_edge":False,"to_time":1.1,"from_scan_id":2},
           {"loop":False,"new_edge":True,"to_time":1.2,"from_scan_id":3},
           {"loop":True,"new_edge":True,"to_time":2,"from_scan_id":4}]
    monitor.feed([(2,2,0)],[(1.5,3,4.5)],edges,
        [{"time":1,"driver_loop_gap_wall_s":.02},{"time":2,"driver_loop_gap_wall_s":.8}])
    assert monitor.first["error_m"] == pytest.approx(1)
    assert monitor.first["last_accepted_loop_edge"]["from_scan_id"] == 1
    assert monitor.first["driver_max_gap_wall_s_before_crossing"] == .02
    with pytest.raises(ValueError,match="decreased"):
        monitor.feed([(0,0,0)])
    with pytest.raises(ValueError,match="cover"):
        monitor.gt_at(-1)


def test_offline_replay_ignores_native_shutdown_partial_frames(tmp_path):
    (tmp_path/"physics-truth.csv").write_text("time,x,y\n0,0,0\n1,1,0\n1")
    (tmp_path/"slam-pose.csv").write_text("time,x,y,yaw\n0.5,0.9,0,0\n0.")
    (tmp_path/"karto-edges.jsonl").write_text(
        '{"loop":true,"new_edge":true,"from_scan_id":1,"to_scan_id":2,"to_time":0.2}\n{"loop":')
    (tmp_path/"driver-trace.jsonl").write_text(
        '{"time":0.5,"driver_loop_gap_wall_s":0.02}\n{"time":')
    result=replay_pose(tmp_path,[0,0,0])
    assert result["max_pose_error_m"] == pytest.approx(.4)
    first=result["first_error_over_0_3m"]
    assert first["sim_time"] == .5
    assert first["last_accepted_loop_edge"]["from_scan_id"] == 1
    assert first["driver_max_gap_wall_s_before_crossing"] == .02
    with (tmp_path/"physics-truth.csv").open("a") as stream:
        stream.write("\n")
    with pytest.raises(TypeError):
        replay_pose(tmp_path,[0,0,0])
