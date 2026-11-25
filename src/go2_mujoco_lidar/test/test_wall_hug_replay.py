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
import copy
import math
import hashlib
import json
import numpy as np
from replay import (ROOT as FIXTURE_ROOT, DEFAULT_DATASET, DatasetAbsent,
    checked_dataset, classify, comparison, verify, load_dataset, candidate_for,
    line_support, EXPECTED_COUNTS, truth)


@pytest.fixture(scope="module")
def measured():
    return classify(json.loads((FIXTURE_ROOT/"fixtures/measurements.json").read_text()))


def test_first_wrong_edge_from_relative_measurement(measured):
    summary=comparison(measured)
    verify(summary)
    first=next(r for r in measured if not r["correct"])
    assert first["relative_xy_error_m"] == pytest.approx(3.5538112158434134)
    assert first["relative_yaw_error_deg"] < 1
    # A mutation that substitutes the GT measurement must change the verdict.
    from replay import relative
    changed=copy.deepcopy(measured[:1])
    changed[0]["relative_pose"]=relative(np.array(first["gt_from_pose"]),np.array(first["gt_to_pose"])).tolist()
    assert classify(changed)[0]["correct"]


@pytest.mark.parametrize("name,expected", list(EXPECTED_COUNTS.items()))
def test_report_gate_counts(measured,name,expected):
    profile=next(p for p in comparison(measured)["profiles"] if p["name"]==name)
    assert (profile["wrong_rejected"],profile["correct_lost"]) == expected


def test_recorded_dataset_replay_matches_small_projection(measured):
    try:
        rows=load_dataset(DEFAULT_DATASET)
    except DatasetAbsent as error:
        pytest.skip(str(error))
    verify(comparison(rows))
    assert len(rows)==len(measured)
    for actual, fixture in zip(rows,measured):
        for key in ("attempt","from_id","to_id","time","correct","chain_size"):
            assert actual[key]==fixture[key]
        for key in ("relative_pose","gt_from_pose","gt_to_pose","coarse_covariance",
                    "implied_correction_m","path_cumulative_m","distance_since_actual_loop_m"):
            assert np.allclose(actual[key],fixture[key],rtol=0,atol=1e-6),key
        assert actual["fine_link_covariance"] == fixture["fine_link_covariance"]
        for side in ("from_geometry","to_geometry"):
            assert actual[side]["sensor_only"]["dominant_line_fraction"] == fixture[side]["sensor_only"]["dominant_line_fraction"]


def test_missing_dataset_and_sha_tamper_are_distinct(tmp_path):
    manifest={"scan.jsonl":hashlib.sha256(b"original").hexdigest()}
    with pytest.raises(DatasetAbsent,match="pinned AGX path"):
        checked_dataset(tmp_path,manifest)
    (tmp_path/"scan.jsonl").write_bytes(b"modified")
    with pytest.raises(ValueError,match="SHA256 mismatch"):
        checked_dataset(tmp_path,manifest)
    (tmp_path/"scan.jsonl").write_bytes(b"original")
    checked_dataset(tmp_path,manifest)


@pytest.mark.parametrize("mutation",["chain_size","covariance","anchor","pose","duplicate"])
def test_native_candidate_identity_and_matrix_contract(mutation):
    edge={"from_scan_id":140,"to_scan_id":275,"match_sensor_pose":[1,2,.3]}
    c={"scan_id":275,"fine_sensor_pose":[1,2,.3],"chain_size":2,
        "coarse_covariance":[1,0,0,0,1,0,0,0,1],
        "chain":[{"scan_id":140,"pose":[0,0,0]},{"scan_id":141,"pose":[.1,0,0]}]}
    assert candidate_for(edge,[c])[1]==[0,0,0]
    if mutation=="chain_size": c["chain_size"]=3
    if mutation=="covariance": c["coarse_covariance"].pop()
    if mutation=="anchor": c["chain"][0]["scan_id"]=1
    if mutation=="pose": c["fine_sensor_pose"][0]+=.01
    if mutation=="duplicate": c["chain"][1]["scan_id"]=140
    with pytest.raises(ValueError):
        candidate_for(edge,[c])


def test_sensor_line_support_from_endpoints():
    angles=np.linspace(-.4,.4,720)
    wall={"scan_id":1,"angle_min":float(angles[0]),"angle_increment":float(angles[1]-angles[0]),
        "range_min":.25,"range_max":5.,"ranges":(.4/np.cos(angles)).tolist()}
    assert line_support(wall)["dominant_line_fraction"] == 1
    assert line_support(wall)["near_wall_degenerate"]
    wall["angle_min"]=-math.pi
    wall["angle_increment"]=2*math.pi/720
    wall["ranges"]=[.4]*720
    assert not line_support(wall)["near_wall_degenerate"]


def test_partial_native_csv_frame_is_not_consumed(tmp_path):
    path=tmp_path/"truth.csv"
    path.write_text("time,x,y,yaw\n0,1,2,0\n.002,3,")
    assert truth(path).tolist()==[[0,1,2,0]]
