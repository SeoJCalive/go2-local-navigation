"""Run isolated Karto/Ceres counterfactuals and evaluate the resulting graph/map.

Each engine process starts from raw recorded ranges and original odometry. GT
is loaded only after the process exits. No prior/pose/edge hint is injected.
The manifest pins all numerical inputs. Baselines precede the gate sweep, with
an explicit attempt-6 reproduction check. Outputs live outside tracked source.
See README.md for correctness, lost-edge and simulation-GT definitions.
진단 확장은 선택적 관측/ROS override와 고정 world→map transform만 전달한다.
기존 12개 manifest·기본 gate·지도 평가식과 제품 parameter는 유지한다.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import heapq
import json
import math
import os
from pathlib import Path
import subprocess
import numpy as np

import geometry
import replay

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parents[3]
MANIFEST = ROOT / "fixtures/counterfactual-datasets.json"


def gates():
    return [dict(name="none", x=0., k=-1., c=0.)] + [
        dict(name=f"distance-{x:g}", x=x, k=-1., c=0.) for x in (1., 1.5, 2., 3.)
    ] + [
        dict(name=f"correction-{k:g}-{c:g}", x=0., k=k, c=c)
        for k in (.03, .05, .10) for c in (.1, .2, .3)
    ]


def motion_robust_gates():
    """Requested reference, four current-graph motion bounds and installed losses."""
    return [dict(name="correction-0.05-0.2", x=0., k=.05, c=.2)] + [
        dict(name=f"motion-{k:g}-{a:g}", x=0., k=k, c=.2, a=a, b=2.)
        for k in (.03, .05) for a in (.5, 1.)
    ] + [
        dict(name=f"robust-{loss}", x=0., k=-1., c=0., loss=loss)
        for loss in ("HuberLoss", "CauchyLoss")
    ]


def validate_motion(decisions, gate):
    """Independent shortest-path and inclusive XY/yaw checks on native snapshots.

    The native path graph is the current chain compressed at retained-loop
    endpoints; its edge weights are refreshed before each candidate. This checks
    the solver separately from the native implementation and catches stale bounds.
    """
    for row in decisions:
        graph = {}
        for source, target, weight in row["path_graph"]:
            if not math.isfinite(weight) or weight < 0.:
                raise ValueError("invalid current graph path weight")
            graph.setdefault(source, []).append((target, weight))
            graph.setdefault(target, []).append((source, weight))
        source, target = row["from_scan_id"], row["to_scan_id"]
        best, queue = {source: 0.}, [(0., source)]
        while queue:
            distance, vertex = heapq.heappop(queue)
            if distance != best[vertex]:
                continue
            if vertex == target:
                break
            for neighbor, weight in graph[vertex]:
                candidate = distance + weight
                if candidate < best.get(neighbor, math.inf):
                    best[neighbor] = candidate
                    heapq.heappush(queue, (candidate, neighbor))
        distance = best[target]
        theta = math.radians(gate["a"] * distance + gate["b"])
        bound = gate["k"] * distance + gate["c"] + distance * theta
        if (abs(row["graph_distance_m"] - distance) > 1e-7 or
                abs(row["xy_bound_m"] - bound) > 1e-7 or
                abs(row["yaw_bound_rad"] - theta) > 1e-7):
            raise ValueError("native current graph distance/bounds mismatch")
        expected = row["correction_m"] <= bound and abs(row["yaw_correction_rad"]) <= theta
        if row["accept"] != expected:
            raise ValueError("native motion decision mismatch")


def checked_inputs(project, manifest):
    for name, expected in manifest["files"].items():
        path = project / name
        if not path.is_file():
            raise replay.DatasetAbsent(f"counterfactual input missing: {path}")
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        if digest.hexdigest() != expected:
            raise ValueError(f"counterfactual SHA256 mismatch: {path}")


def classify_factors(edges, factors, truth):
    by_pair = {(f["from_scan_id"], f["to_scan_id"]): f for f in factors}
    result = []
    for edge in edges:
        if not edge["loop"] or not edge["new_edge"]:
            continue
        pair = edge["from_scan_id"], edge["to_scan_id"]
        factor = by_pair[pair]
        expected = replay.relative(replay.at(truth, edge["from_time"]),
                                   replay.at(truth, edge["to_time"]))
        observed = np.array(factor["relative_pose"])
        xy = float(np.linalg.norm(expected[:2] - observed[:2]))
        yaw = abs(math.degrees(replay.wrap(expected[2] - observed[2])))
        result.append(dict(pair=list(pair), time=edge["to_time"],
                           correct=xy <= .10 and yaw <= 5.,
                           xy_error_m=xy, yaw_error_deg=yaw))
    return result


def pose_error(rows, truth, percentile=100):
    return float(np.percentile([
        float(np.linalg.norm(np.array(r["pose"][:2]) -
                             replay.at(truth, r["time"])[:2]))
        for r in rows if r.get("accepted", True)], percentile))


def dataset_truth(project, dataset, inputs):
    """Use recorded truth and its fixed startup transform; never fit final poses."""
    if dataset["truth"] == "serialized-simulation-odometry":
        truth = np.array([[r["time"], *r["odom"]] for r in inputs])
        truth[:, 3] = np.unwrap(truth[:, 3])
    else:
        truth = replay.truth(project / dataset["truth"])
    yaw, tx, ty = dataset.get("world_to_map", (0., 0., 0.))
    c, s = math.cos(yaw), math.sin(yaw)
    truth[:, 1:3] = truth[:, 1:3] @ np.array([[c, s], [-s, c]]) + [tx, ty]
    truth[:, 3] += yaw
    return truth


def lost_correct(baseline, current):
    """Exact baseline correct edge pairs absent from the counterfactual graph.

    These are absent opportunities, not necessarily explicitly rejected edges:
    new matching may select a different correct anchor. Report new correct count too.
    """
    retained = {tuple(r["pair"]) for r in current if r["correct"]}
    return [r["pair"] for r in baseline if r["correct"] and tuple(r["pair"]) not in retained]


def validate_retained_arc(inputs, online, edges, decisions):
    """Cross-check native D against original odometry and actual new loop edges."""
    source_arc = {r["source_id"]: r["arc_m"] for r in inputs}
    arc = {r["id"]: source_arc[r["source_id"]] for r in online if r["accepted"]}
    inserted = {(e["from_scan_id"], e["to_scan_id"]) for e in edges
                if e["loop"] and e["new_edge"]}
    seen = set()
    anchor = 0.
    for row in decisions:
        current = arc[row["to_scan_id"]]
        if abs(row["distance_since_retained_m"] - (current - anchor)) > 1e-7:
            raise ValueError("native D does not use the last retained new loop")
        pair = row["from_scan_id"], row["to_scan_id"]
        if row["accept"] and pair in inserted and pair not in seen:
            anchor = current
            seen.add(pair)
    if seen != inserted:
        raise ValueError("actual new loop has no matching retained gate decision")


def reproduction(summary):
    wrong = [r for r in summary["loops"] if not r["correct"]]
    # Similarity bands are declared, not exact frozen edge-list requirements.
    # Parallel historical optimization can diverge at a late tie.
    checks = dict(
        first_wrong=bool(wrong and wrong[0]["pair"] == [140, 275]
                         and abs(wrong[0]["time"] - 201.302) < .001),
        wall_p95=abs(summary["wall_offset_p95_m"] - 1.0626808162324566) <= .22,
        doubled=abs(summary["doubled_walls"] - 9) <= 4,
        ghosts=abs(summary["ghost_components"] - 75) <= 20,
        pose_max=abs(summary["pose_error_max_m"] - 5.0277695078040106) <= .55)
    if not all(checks.values()):
        raise ValueError(f"gate=none attempt-6 reproduction failed: {checks}; "
                         "investigate before sweeping gates")
    return checks


def run_one(engine, project, manifest, dataset, gate, output):
    directory = output / dataset["name"] / gate["name"]
    directory.mkdir(parents=True, exist_ok=False)
    prefix = directory / "result"
    command = [str(engine), str(project / dataset["data"]),
               str(project / manifest["parameters"]), str(prefix),
               str(gate["x"]), str(gate["k"]), str(gate["c"]), "0"]
    if "a" in gate or "loss" in gate:
        command.extend([str(gate.get("a", -1.)), str(gate.get("b", 0.)),
                        gate.get("loss", "None")])
    environment = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1",
                       ROS_DOMAIN_ID="198", ROS_LOCALHOST_ONLY="1")
    if gate["name"] == "none" and dataset["name"] in (
            "sep-horizontal", "sep-wallfullclean", "sep-wallboth"):
        environment["GO2_REPLAY_SCAN_EVIDENCE"] = str(prefix) + "-scans.jsonl"
    if gate.get("observations"):
        environment["GO2_REPLAY_SCAN_EVIDENCE"] = str(prefix) + "-scans.jsonl"
        environment["GO2_REPLAY_CANDIDATE_EVIDENCE"] = str(prefix) + "-candidates.jsonl"
    if gate.get("arc") == "pair":
        environment["GO2_REPLAY_ARC"] = "pair"
    if gate.get("parameters"):
        environment["GO2_REPLAY_ROS_ARGS"] = " ".join(
            f"-p {name}:={json.dumps(value)}" for name, value in gate["parameters"].items())
    # Inherit ROS loader paths but never any recorded-prior preload.
    if environment.get("LD_PRELOAD"):
        raise ValueError("LD_PRELOAD must be unset for causal replay")
    with (directory / "engine.log").open("w") as log:
        subprocess.run(command, env=environment, stdout=log, stderr=subprocess.STDOUT,
                       check=True, timeout=1200)
    inputs = list(replay.records(Path(str(prefix) + "-inputs.jsonl")))
    truth = dataset_truth(project, dataset, inputs)
    if inputs[0]["time"] < truth[0, 0] or inputs[-1]["time"] > truth[-1, 0]:
        raise ValueError("ground truth does not bracket all input scans")
    loops = classify_factors(
        replay.records(Path(str(prefix) + "-edges.jsonl")),
        replay.records(Path(str(prefix) + "-factors.jsonl")), truth)
    online = list(replay.records(Path(str(prefix) + "-online.jsonl")))
    validate_retained_arc(inputs, online,
                         replay.records(Path(str(prefix) + "-edges.jsonl")),
                         replay.records(Path(str(prefix) + "-decisions.jsonl")))
    if "a" in gate:
        validate_motion(replay.records(Path(str(prefix) + "-decisions.jsonl")), gate)
    final = list(replay.records(Path(str(prefix) + "-vertices.jsonl")))
    # Map coordinates are initialized from the input world-coordinate odometry.
    # No fitted final transform can hide a bad closure.
    metrics = geometry.evaluate(project / manifest["world"], str(prefix) + ".yaml",
                                dataset.get("world_to_map", (0., 0., 0.)),
                                directory / "gt-overlay.png")
    (directory / "map-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    summary = dict(dataset=dataset["name"], gate=gate, truth=dataset["truth"],
                   inputs=len(inputs), accepted_scans=len(final),
                   loops=loops, wrong_loops=sum(not r["correct"] for r in loops),
                   wrong_loops_gt_1m=sum(not r["correct"] and r["xy_error_m"] > 1.
                                        for r in loops),
                   correct_loops=sum(r["correct"] for r in loops),
                   wall_offset_p95_m=metrics["wall_offset_m"]["p95"],
                   doubled_walls=metrics["doubled_wall_components"],
                   ghost_components=metrics["ghost_wall_components"],
                   pose_error_max_m=pose_error(online, truth),
                   pose_error_p95_m=pose_error(online, truth, 95),
                   final_graph_pose_error_max_m=pose_error(final, truth),
                   command=command)
    (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"REPLAY_DONE {dataset['name']} {gate['name']} "
          f"wrong={summary['wrong_loops']} correct={summary['correct_loops']} "
          f"p95={summary['wall_offset_p95_m']:.6f}", flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--project", type=Path, default=PROJECT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=2)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--baseline", action="store_true")
    mode.add_argument("--sweep", action="store_true")
    mode.add_argument("--combined", nargs=3, type=float, metavar=("X", "K", "C"))
    mode.add_argument("--motion-robust", action="store_true")
    mode.add_argument("--motion-combined", nargs=3, metavar=("K", "A", "LOSS"))
    parser.add_argument("--dataset", action="append")
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    checked_inputs(args.project, manifest)
    datasets = [d for d in manifest["datasets"]
                if not args.dataset or d["name"] in args.dataset]
    selected = gates()[:1] if args.baseline else gates()[1:]
    if args.combined:
        x, k, c = args.combined
        selected = [dict(name=f"combined-{x:g}-{k:g}-{c:g}", x=x, k=k, c=c)]
    if args.motion_robust:
        selected = motion_robust_gates()
    if args.motion_combined:
        k, a = map(float, args.motion_combined[:2])
        loss = args.motion_combined[2]
        if k not in (.03, .05) or a not in (.5, 1.) or loss not in ("HuberLoss", "CauchyLoss"):
            parser.error("combined variant must use the requested grid and installed losses")
        selected = [dict(name=f"motion-{k:g}-{a:g}-{loss}", x=0., k=k, c=.2,
                         a=a, b=2., loss=loss)]
    if not args.baseline:
        base = args.output / "g02-6/none/summary.json"
        reproduction(json.loads(base.read_text()))
    args.output.mkdir(parents=True, exist_ok=True)
    work = [(d, g) for d in datasets for g in selected]
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(lambda item: run_one(args.engine, args.project, manifest,
                                                    *item, args.output), work))
    if args.baseline and any(r["dataset"] == "g02-6" for r in results):
        reproduction(next(r for r in results if r["dataset"] == "g02-6"))
    for row in results:
        baseline = json.loads((args.output / row["dataset"] / "none/summary.json").read_text())
        row["correct_loops_lost"] = lost_correct(baseline["loops"], row["loops"])
    table = args.output / ("motion-robust.json" if args.motion_robust else
                           "motion-combined.json" if args.motion_combined else
                           "baseline.json" if args.baseline else
                           "combined.json" if args.combined else "sweep.json")
    table.write_text(json.dumps(results, indent=2) + "\n")
    if args.motion_robust and not args.dataset:
        # Preserve the original paper's selection order, on its original 10 inputs.
        # September 21 is a held-out challenge cohort, reported separately.
        original = {d["name"] for d in manifest["datasets"] if d.get("cohort") != "supplemental"}
        candidates = {}
        for row in results:
            if row["dataset"] in original:
                candidates.setdefault(row["gate"]["name"], []).append(row)
        def rank(rows):
            g02 = next(r for r in rows if r["dataset"] == "g02-6")
            gate = rows[0]["gate"]
            return (sum(r["wrong_loops"] for r in rows), g02["pose_error_max_m"],
                    g02["wall_offset_p95_m"], -sum(r["correct_loops"] for r in rows),
                    -gate.get("k", 0.), -gate.get("a", 0.), gate["name"])
        motion = min((r for r in candidates.values() if "a" in r[0]["gate"]), key=rank)
        robust = min((r for r in candidates.values() if "loss" in r[0]["gate"]), key=rank)
        selection = dict(
            selection_cohort=sorted(original),
            rule="wrong sum, G02 online max, G02 p95, negative correct total; then larger k/a",
            best_motion=motion[0]["gate"], best_robust=robust[0]["gate"])
        (args.output / "motion-selection.json").write_text(json.dumps(selection, indent=2) + "\n")
        print("MOTION_SELECTION " + json.dumps(selection), flush=True)
    if args.motion_combined:
        rows = []
        for filename in ("baseline.json", "motion-robust.json", "motion-combined.json"):
            rows.extend(json.loads((args.output / filename).read_text()))
        supplementary = {d["name"] for d in manifest["datasets"]
                         if d.get("cohort") == "supplemental"}
        cohorts = {
            "original_10": {d["name"] for d in manifest["datasets"]} - supplementary,
            "supplementary_2": supplementary,
            "all_12": {d["name"] for d in manifest["datasets"]}}
        aggregate = {}
        for cohort, names in cohorts.items():
            groups = {}
            for row in rows:
                if row["dataset"] in names:
                    groups.setdefault(row["gate"]["name"], []).append(row)
            aggregate[cohort] = [
                dict(name=name, datasets=len(group),
                     wrong=sum(r["wrong_loops"] for r in group),
                     wrong_xy_over_1m=sum(r["wrong_loops_gt_1m"] for r in group),
                     correct=sum(r["correct_loops"] for r in group),
                     correct_pair_absent=sum(len(r["correct_loops_lost"]) for r in group),
                     p95_max=max(r["wall_offset_p95_m"] for r in group),
                     doubled_sum=sum(r["doubled_walls"] for r in group),
                     ghost_sum=sum(r["ghost_components"] for r in group),
                     pose_max=max(r["pose_error_max_m"] for r in group))
                for name, group in groups.items()]
        (args.output / "motion-comparison.json").write_text(
            json.dumps(dict(aggregate=aggregate, rows=rows), indent=2) + "\n")
    print("COUNTERFACTUAL_COMPLETE " + str(table), flush=True)


if __name__ == "__main__":
    main()
