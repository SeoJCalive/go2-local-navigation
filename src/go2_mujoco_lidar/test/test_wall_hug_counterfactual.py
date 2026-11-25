"""Counterfactual contracts plus a real raw-scan causal/determinism regression.

Numerical unit controls always run. The native regression requires the separately
built, repo-only pinned engine and recorded AGX files, and otherwise reports skip.
No robot, GUI, ROS executor, live scan source or system installation is started.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "wall_hug"))
import counterfactual_replay as cf
import replay


def test_complete_gate_grid_sigma_is_not_a_sweep_parameter():
    grid = cf.gates()
    assert len(grid) == 14
    assert len({g["name"] for g in grid}) == 14
    assert [g["x"] for g in grid[1:5]] == [1., 1.5, 2., 3.]
    assert {(g["k"], g["c"]) for g in grid[5:]} == {
        (k, c) for k in (.03, .05, .1) for c in (.1, .2, .3)}


def test_motion_grid_and_installed_loss_variants():
    grid = cf.motion_robust_gates()
    assert len(grid) == len({g["name"] for g in grid}) == 7
    assert {(g["k"], g["a"], g["b"], g["c"]) for g in grid if "a" in g} == {
        (k, a, 2., .2) for k in (.03, .05) for a in (.5, 1.)}
    assert {g["loss"] for g in grid if "loss" in g} == {"HuberLoss", "CauchyLoss"}
    assert all(g["k"] < 0 and not g["x"] for g in grid if "loss" in g)


def test_current_graph_shortcut_and_independent_inclusive_yaw_bound():
    gate = dict(k=.05, c=.2, a=.5, b=2.)
    theta = math.radians(2.5)
    bound = .05 + .2 + theta
    row = dict(from_scan_id=0, to_scan_id=2,
               path_graph=[[0, 1, 10.], [1, 2, 10.], [0, 2, 1.]],
               graph_distance_m=1., xy_bound_m=bound, yaw_bound_rad=theta,
               correction_m=bound, yaw_correction_rad=-theta, accept=True)
    cf.validate_motion([row], gate)
    row["yaw_correction_rad"] = theta + 1e-5
    row["accept"] = False
    cf.validate_motion([row], gate)
    row["graph_distance_m"] = 20.
    with pytest.raises(ValueError, match="graph distance/bounds"):
        cf.validate_motion([row], gate)


def test_hash_pin_missing_and_corrupt_are_distinct(tmp_path):
    manifest = {"files": {"data": hashlib.sha256(b"raw").hexdigest()}}
    with pytest.raises(replay.DatasetAbsent):
        cf.checked_inputs(tmp_path, manifest)
    (tmp_path / "data").write_bytes(b"edited")
    with pytest.raises(ValueError, match="SHA256"):
        cf.checked_inputs(tmp_path, manifest)
    (tmp_path / "data").write_bytes(b"raw")
    cf.checked_inputs(tmp_path, manifest)


def test_relative_measurement_not_node_distance_defines_correctness():
    truth = np.array([[0., 0., 0., 0.], [1., 3., 0., 0.]])
    edge = dict(from_scan_id=0, to_scan_id=1, from_time=0., to_time=1.,
                loop=True, new_edge=True)
    factor = dict(from_scan_id=0, to_scan_id=1, relative_pose=[3., 0., 0.])
    assert cf.classify_factors([edge], [factor], truth)[0]["correct"]
    factor["relative_pose"] = [.5, 0., 0.]
    assert not cf.classify_factors([edge], [factor], truth)[0]["correct"]
    edge["new_edge"] = False
    assert not cf.classify_factors([edge], [factor], truth)


def test_absent_baseline_pair_is_distinct_from_new_correct_alternative():
    baseline = [dict(pair=[1, 100], correct=True), dict(pair=[2, 200], correct=False)]
    changed = [dict(pair=[3, 100], correct=True)]
    assert cf.lost_correct(baseline, changed) == [[1, 100]]
    assert cf.lost_correct(baseline, baseline) == []


def test_reproduction_requires_first_error_and_similar_map_and_pose():
    summary = dict(loops=[dict(pair=[140, 275], time=201.302, correct=False)],
                   wall_offset_p95_m=1.063, doubled_walls=9, ghost_components=75,
                   pose_error_max_m=5.028)
    assert all(cf.reproduction(summary).values())
    summary["ghost_components"] = 0
    with pytest.raises(ValueError, match="investigate before sweeping"):
        cf.reproduction(summary)


def test_only_new_inserted_loops_reset_arc_not_rejections_or_callbacks():
    inputs = [dict(source_id=i, arc_m=a) for i, a in enumerate((0., 10., 20., 30., 40.))]
    online = [dict(source_id=i, id=i, accepted=True) for i in range(5)]
    edges = [dict(from_scan_id=0, to_scan_id=2, loop=True, new_edge=True)]
    decisions = [
        dict(from_scan_id=0, to_scan_id=1, accept=False, distance_since_retained_m=10.),
        dict(from_scan_id=0, to_scan_id=2, accept=True, distance_since_retained_m=20.),
        dict(from_scan_id=0, to_scan_id=2, accept=True, distance_since_retained_m=0.),
        dict(from_scan_id=1, to_scan_id=3, accept=True, distance_since_retained_m=10.),
        dict(from_scan_id=1, to_scan_id=4, accept=False, distance_since_retained_m=20.)]
    cf.validate_retained_arc(inputs, online, edges, decisions)
    decisions[-1]["distance_since_retained_m"] = 0.
    with pytest.raises(ValueError, match="last retained"):
        cf.validate_retained_arc(inputs, online, edges, decisions)


def test_native_reoptimization_and_repeatability(tmp_path):
    engine = Path(os.environ.get("GO2_COUNTERFACTUAL_ENGINE", str(
        cf.PROJECT / ".omo/evidence/loop-gate-replay/engine-motion-v2/build/engine_replay")))
    manifest = json.loads(cf.MANIFEST.read_text())
    if not engine.is_file():
        pytest.skip("repo-only Karto engine absent; build counterfactual/build.sh first")
    try:
        cf.checked_inputs(cf.PROJECT, manifest)
    except replay.DatasetAbsent as error:
        pytest.skip(str(error))
    data = cf.PROJECT / manifest["datasets"][0]["data"]
    params = cf.PROJECT / manifest["parameters"]
    environment = dict(os.environ, ROS_DOMAIN_ID="198", ROS_LOCALHOST_ONLY="1",
                       OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    assert not environment.get("LD_PRELOAD"), "recorded-prior injection is forbidden"
    for name, x, k, c in (
        ("none", 0, -1, 0), ("strict-a", 1.5, -1, 0), ("strict-b", 1.5, -1, 0),
        ("correction-strict", 0, .03, .1), ("correction-loose", 0, .1, .3)
    ):
        prefix = tmp_path / name
        with (tmp_path / (name + ".log")).open("w") as log:
            subprocess.run([str(engine), str(data), str(params), str(prefix),
                            str(x), str(k), str(c), "350"], env=environment,
                           stdout=log, stderr=subprocess.STDOUT, timeout=90, check=True)
        cf.validate_retained_arc(
            replay.records(Path(str(prefix) + "-inputs.jsonl")),
            replay.records(Path(str(prefix) + "-online.jsonl")),
            replay.records(Path(str(prefix) + "-edges.jsonl")),
            replay.records(Path(str(prefix) + "-decisions.jsonl")))
    original = list(replay.records(tmp_path / "none-edges.jsonl"))
    original_loops = [e for e in original if e["loop"] and e["new_edge"]]
    assert [(e["from_scan_id"], e["to_scan_id"], e["to_time"])
            for e in original_loops] == [(140, 275, 201.302)]
    changed = list(replay.records(tmp_path / "strict-a-edges.jsonl"))
    assert not any(e["loop"] and e["new_edge"] and e["to_scan_id"] == 275 for e in changed)
    assert any(e["loop"] and e["new_edge"] and e["to_scan_id"] == 328 for e in changed)
    rejected = list(replay.records(tmp_path / "strict-a-decisions.jsonl"))
    assert any(r["to_scan_id"] == 275 and r["separation_m"] > 1.5 and not r["accept"]
               for r in rejected)
    # Verify downstream engine state changes; this cannot pass with a frozen-edge re-judge.
    a = list(replay.records(tmp_path / "none-online.jsonl"))[-1]["pose"]
    b = list(replay.records(tmp_path / "strict-a-online.jsonl"))[-1]["pose"]
    assert np.linalg.norm(np.array(a[:2]) - b[:2]) > 1.
    for name, expected in (("correction-strict", False), ("correction-loose", True)):
        rows = list(replay.records(tmp_path / (name + "-decisions.jsonl")))
        first = next(r for r in rows if r["to_scan_id"] == 275)
        assert first["from_scan_id"] == 140
        assert first["distance_since_retained_m"] == pytest.approx(35.8524, abs=.0001)
        assert first["correction_m"] == pytest.approx(3.5600, abs=.0001)
        assert first["accept"] is expected
        inserted = [r for r in replay.records(tmp_path / (name + "-edges.jsonl"))
                    if r["loop"] and r["new_edge"] and r["to_scan_id"] == 275]
        assert bool(inserted) is expected
    for suffix in ("-edges.jsonl", "-factors.jsonl", "-online.jsonl",
                   "-vertices.jsonl", "-decisions.jsonl", ".pgm"):
        assert (tmp_path / ("strict-a" + suffix)).read_bytes() == (
            tmp_path / ("strict-b" + suffix)).read_bytes(), suffix


@pytest.mark.parametrize("gate", cf.motion_robust_gates()[1:] + [
    dict(name="motion-0.03-0.5-CauchyLoss", x=0., k=.03, c=.2,
         a=.5, b=2., loss="CauchyLoss")], ids=lambda gate: gate["name"])
def test_native_motion_robust_variants_are_deterministic(tmp_path, gate):
    """Every new grid/loss variant really rematches and optimizes pinned raw data."""
    engine = Path(os.environ.get("GO2_COUNTERFACTUAL_ENGINE", str(
        cf.PROJECT / ".omo/evidence/loop-gate-replay/engine-motion-v2/build/engine_replay")))
    manifest = json.loads(cf.MANIFEST.read_text())
    if not engine.is_file():
        pytest.skip("repo-only v2 engine absent; build into engine-motion-v2 first")
    try:
        cf.checked_inputs(cf.PROJECT, manifest)
    except replay.DatasetAbsent as error:
        pytest.skip(str(error))
    environment = dict(os.environ, ROS_DOMAIN_ID="198", ROS_LOCALHOST_ONLY="1",
                       OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    assert not environment.get("LD_PRELOAD")
    # G02's first 350 inputs contain just one candidate with these permissive
    # motion bounds. The archived September run closes an earlier loop before
    # subsequent candidates, exercising a graph with real retained loop edges.
    dataset = next(d for d in manifest["datasets"] if d["name"] == (
        "sep21-1800" if "a" in gate else "g02-6"))
    for repeat in ("a", "b"):
        prefix = tmp_path / repeat
        with (tmp_path / (repeat + ".log")).open("w") as log:
            subprocess.run([
                str(engine), str(cf.PROJECT / dataset["data"]),
                str(cf.PROJECT / manifest["parameters"]), str(prefix),
                str(gate["x"]), str(gate["k"]), str(gate["c"]), "350",
                str(gate.get("a", -1.)), str(gate.get("b", 0.)),
                gate.get("loss", "None")], env=environment, stdout=log,
                stderr=subprocess.STDOUT, check=True, timeout=90)
        rows = list(replay.records(Path(str(prefix) + "-decisions.jsonl")))
        assert rows, "the native regression must exercise loop candidates"
        cf.validate_retained_arc(
            replay.records(Path(str(prefix) + "-inputs.jsonl")),
            replay.records(Path(str(prefix) + "-online.jsonl")),
            replay.records(Path(str(prefix) + "-edges.jsonl")), rows)
        if "a" in gate:
            cf.validate_motion(rows, gate)
            assert all(r["graph_distance_m"] <= r["chain_distance_m"] + 1e-7 for r in rows)
            assert any(r["retained_loops"] for r in rows), "exercise a post-loop graph"
        if "loss" in gate:
            assert f"Using {gate['loss']} loss function" in (
                tmp_path / (repeat + ".log")).read_text()
    for suffix in ("-edges.jsonl", "-factors.jsonl", "-online.jsonl",
                   "-vertices.jsonl", "-decisions.jsonl", ".pgm"):
        assert (tmp_path / ("a" + suffix)).read_bytes() == (
            tmp_path / ("b" + suffix)).read_bytes(), (gate["name"], suffix)
