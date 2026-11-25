"""목적: 13번째 cohort·pair arc·진단 override의 경계를 검사한다.

입력은 고정 fixture와 선택적 실제 replay engine이다. GT를 native에 주입하지 않고
inclusive gate·원본 arc·기본 호환·matching-off 대조를 확인한다. GUI는 실행하지 않는다.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "wall_hug"))
import counterfactual_replay as cf
import drift_replay as drift
import replay


def test_thirteenth_dataset_keeps_original_manifest_and_cohorts():
    original = json.loads(cf.MANIFEST.read_text())
    extended = drift.manifest()
    assert len(original["datasets"]) == 12
    assert len(extended["datasets"]) == 13
    assert extended["datasets"][:12] == original["datasets"]
    assert extended["datasets"][-1]["name"] == "g02-9"
    assert all(extended["files"][key] == value for key, value in original["files"].items())


def test_requested_pair_grid_is_distinct_from_retained_reference():
    gates = drift.gates()
    assert len(gates) == len({r["name"] for r in gates}) == 9
    assert {(r["k"], r["c"]) for r in gates if r.get("arc") == "pair"} == {
        (k, c) for k in (.03, .05, .08) for c in (.2, .5)}
    assert gates[-1]["name"] == "retained-0.05-0.2" and "arc" not in gates[-1]


def test_sensor_to_base_preserves_nonzero_extrinsics():
    offset = [.13, -.04, .3]
    base = np.array([1., 2., .6])
    c, s = np.cos(base[2]), np.sin(base[2])
    sensor = [base[0]+c*offset[0]-s*offset[1],
              base[1]+s*offset[0]+c*offset[1], base[2]+offset[2]]
    np.testing.assert_allclose(drift.base_pose(sensor, offset), base, atol=1e-15)


def test_fine_resolution_uses_native_valid_smear_and_matched_control():
    variants = {r["name"]: r["parameters"] for r in drift.sensitivities()}
    fine = variants["search-fine-smear03"]
    assert fine["correlation_search_space_resolution"] == .005
    # Actual native rejection of .005/.1 declared the allowed [.0025,.05] interval.
    assert .0025 <= fine["correlation_search_space_smear_deviation"] <= .05
    assert fine["correlation_search_space_smear_deviation"] == variants[
        "smear-fine"]["correlation_search_space_smear_deviation"]


def test_absent_pairs_use_source_ids_after_movement_filter(tmp_path, monkeypatch):
    baseline = dict(loops=[dict(pair=[0, 2], correct=True)])
    row = dict(dataset="filtered", gate=dict(name="coarse"),
               loops=[dict(pair=[0, 1], correct=True)])
    for name, item, sources in (
            ("none", baseline, [(0, 10), (2, 30)]),
            ("coarse", row, [(0, 10), (1, 30)])):
        directory = tmp_path / "filtered" / name
        directory.mkdir(parents=True)
        (directory / "summary.json").write_text(json.dumps(item))
        (directory / "result-online.jsonl").write_text("".join(
            json.dumps(dict(id=node, source_id=source, accepted=True)) + "\n"
            for node, source in sources))
    monkeypatch.setattr(drift, "aggregate", lambda rows: {})
    drift.write_results([row], tmp_path, "comparison.json")
    assert row["correct_loops_lost"] == []


def test_native_pair_arc_and_parameter_override(tmp_path):
    engine = Path(os.environ.get("GO2_COUNTERFACTUAL_ENGINE", str(
        cf.PROJECT / ".omo/evidence/loop-gate-drift/engine/engine_replay")))
    if not engine.is_file():
        pytest.skip("build isolated drift replay engine first")
    spec = drift.manifest()
    try:
        cf.checked_inputs(cf.PROJECT, spec)
    except replay.DatasetAbsent as error:
        pytest.skip(str(error))
    dataset = spec["datasets"][0]
    environment = dict(os.environ, ROS_DOMAIN_ID="198", ROS_LOCALHOST_ONLY="1",
                       OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    for name in ("pair-a", "pair-b", "scan-off"):
        env = dict(environment)
        if name.startswith("pair"):
            env["GO2_REPLAY_ARC"] = "pair"
        else:
            env["GO2_REPLAY_ROS_ARGS"] = "-p use_scan_matching:=false"
        prefix = tmp_path / name
        command = [str(engine), str(cf.PROJECT / dataset["data"]),
                   str(cf.PROJECT / spec["parameters"]), str(prefix),
                   "0", ".03", ".2", "350"]
        with (tmp_path / f"{name}.log").open("w") as log:
            subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                           check=True, timeout=120)
        inputs = {r["source_id"]: r for r in replay.records(
            tmp_path / f"{name}-inputs.jsonl")}
        online = list(replay.records(tmp_path / f"{name}-online.jsonl"))
        if name == "scan-off":
            assert all(r["accepted"] and r["pose"] == inputs[r["source_id"]]["odom"]
                       for r in online)
            assert not list(replay.records(tmp_path / f"{name}-edges.jsonl"))
            continue
        arcs = {r["id"]: inputs[r["source_id"]]["arc_m"] for r in online if r["accepted"]}
        decisions = list(replay.records(tmp_path / f"{name}-decisions.jsonl"))
        assert decisions
        first = next(r for r in decisions if r["to_scan_id"] == 275)
        assert first["odometry_pair_arc_m"] < first["distance_since_retained_m"]
        assert not first["accept"]
        for row in decisions:
            distance = arcs[row["to_scan_id"]] - arcs[row["from_scan_id"]]
            assert row["odometry_pair_arc_m"] == pytest.approx(distance, abs=1e-7)
            assert row["xy_bound_m"] == pytest.approx(.03*distance+.2, abs=1e-7)
            assert row["accept"] == (row["correction_m"] <= row["xy_bound_m"])
    for suffix in ("-edges.jsonl", "-factors.jsonl", "-online.jsonl",
                   "-vertices.jsonl", "-decisions.jsonl", ".pgm"):
        assert (tmp_path / ("pair-a" + suffix)).read_bytes() == (
            tmp_path / ("pair-b" + suffix)).read_bytes()
