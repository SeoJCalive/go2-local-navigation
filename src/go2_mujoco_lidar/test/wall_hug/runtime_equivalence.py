"""목적: built runtime SDK의 0/1 m 결과를 고정 counterfactual 결과와 대조한다.

흐름: SHA 입력 확인 → 원본 scan 재생 → 실제 factor/GT 분류 → byte·지표 비교.
입출력: runtime engine·새 output directory → 24개 실행과 equivalence.json.
주의: 기존 worker1 offline solver를 재사용한다. GUI/실물 실행이나 GT 주입은 없다.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess

import numpy as np

import counterfactual_replay as cf
import geometry
import replay


SUFFIXES = ("-edges.jsonl", "-factors.jsonl", "-online.jsonl",
            "-vertices.jsonl", ".pgm")


def reference(project, name, distance):
    root = project / ".omo/evidence/loop-gate-replay"
    if name.startswith("sep21-"):
        return root / "motion-robust" / name / (
            "combined-1--1-0" if distance else "none")
    return root / "runs-isolated" / name / ("distance-1" if distance else "none")


def compare_files(actual, expected):
    for suffix in SUFFIXES:
        if Path(str(actual) + suffix).read_bytes() != Path(str(expected) + suffix).read_bytes():
            raise ValueError(f"runtime/replay bytes differ: {suffix}: {actual}")


def run_one(engine, project, manifest, dataset, distance, output):
    directory = output / dataset["name"] / f"param-{distance:g}"
    directory.mkdir(parents=True, exist_ok=False)
    prefix = directory / "result"
    command = [str(engine), str(project / dataset["data"]),
               str(project / manifest["parameters"]), str(prefix),
               str(distance), "-1", "0", "0"]
    environment = dict(os.environ, ROS_DOMAIN_ID="198", ROS_LOCALHOST_ONLY="1",
                       OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    assert not environment.get("LD_PRELOAD")
    with (directory / "engine.log").open("w") as log:
        subprocess.run(command, env=environment, stdout=log, stderr=subprocess.STDOUT,
                       check=True, timeout=1200)
    inputs = list(replay.records(Path(str(prefix) + "-inputs.jsonl")))
    if dataset["truth"] == "serialized-simulation-odometry":
        truth = np.array([[r["time"], *r["odom"]] for r in inputs])
        truth[:, 3] = np.unwrap(truth[:, 3])
    else:
        truth = replay.truth(project / dataset["truth"])
    loops = cf.classify_factors(replay.records(Path(str(prefix) + "-edges.jsonl")),
                                replay.records(Path(str(prefix) + "-factors.jsonl")), truth)
    metrics = geometry.evaluate(project / manifest["world"], str(prefix) + ".yaml",
                                (0., 0., 0.), directory / "gt-overlay.png")
    summary = dict(dataset=dataset["name"], max_odom_distance_m=distance,
                   wrong_loops=sum(not row["correct"] for row in loops),
                   correct_loops=sum(row["correct"] for row in loops),
                   wall_offset_p95_m=metrics["wall_offset_m"]["p95"],
                   doubled_walls=metrics["doubled_wall_components"],
                   ghost_components=metrics["ghost_wall_components"],
                   pose_error_max_m=cf.pose_error(
                       replay.records(Path(str(prefix) + "-online.jsonl")), truth))
    expected_dir = reference(project, dataset["name"], distance)
    expected = json.loads((expected_dir / "summary.json").read_text())
    compare_files(prefix, expected_dir / "result")
    for key in ("wrong_loops", "correct_loops", "wall_offset_p95_m", "doubled_walls",
                "ghost_components", "pose_error_max_m"):
        if abs(summary[key] - expected[key]) > 1e-8:
            raise ValueError(f"runtime metric mismatch: {key}: {summary[key]} != {expected[key]}")
    if distance and summary["wrong_loops"]:
        raise ValueError("distance 1 m runtime inserted wrong loops")
    summary["reference"] = str(expected_dir)
    summary["exact_outputs"] = list(SUFFIXES)
    (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (directory / "map-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print("RUNTIME_EQUIVALENT " + json.dumps(summary), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=3)
    args = parser.parse_args()
    manifest = json.loads(cf.MANIFEST.read_text())
    cf.checked_inputs(cf.PROJECT, manifest)
    args.output.mkdir(parents=True, exist_ok=False)
    pairs = [(dataset, distance) for dataset in manifest["datasets"] for distance in (0., 1.)]
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        rows = list(pool.map(lambda pair: run_one(
            args.engine, cf.PROJECT, manifest, pair[0], pair[1], args.output), pairs))
    (args.output / "equivalence.json").write_text(json.dumps(rows, indent=2) + "\n")
    print(f"RUNTIME_EQUIVALENCE_PASS runs={len(rows)}", flush=True)


if __name__ == "__main__":
    main()
