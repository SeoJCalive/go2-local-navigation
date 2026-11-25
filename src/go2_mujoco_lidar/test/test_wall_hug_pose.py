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
import json
import math
from pose_watch import Lines, PoseMonitor, prepare_run


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
