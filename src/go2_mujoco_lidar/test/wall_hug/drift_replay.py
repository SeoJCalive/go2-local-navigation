"""목적: 12개 기존 cohort와 attempt9의 gate·matcher 민감도를 분리해 검증한다.

흐름: 고정 raw/GT → 기존 native 재매칭·최적화 → 기존 지도 판정·후보 GT audit.
입출력: engine/새 output → dataset별 원본·13/12/attempt9 집계·구간 drift JSON.
주의: GT는 사후 평가뿐이며 설정·지도 지표·threshold·제품 runtime을 바꾸지 않는다.
pair arc와 과거 retained-loop arc는 서로 다른 정책으로 명시한다.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path

import numpy as np

import counterfactual_replay as cf
import counterfactual_profiles as profiles
import geometry
import replay

HERE = Path(__file__).resolve().parent


def manifest():
    extension = json.loads((HERE / "fixtures/drift-datasets.json").read_text())
    result = json.loads((HERE / "fixtures" / extension["base_manifest"]).read_text())
    result["files"].update(extension["files"])
    result["datasets"].extend(extension["datasets"])
    return result


def gates():
    return [
        dict(name="none", x=0., k=-1., c=0., observations=True),
        dict(name="distance-1", x=1., k=-1., c=0., observations=True),
    ] + [
        dict(name=f"pair-{k:g}-{c:g}", x=0., k=k, c=c,
             arc="pair", observations=True)
        for k in (.03, .05, .08) for c in (.2, .5)
    ] + [
        dict(name="retained-0.05-0.2", x=0., k=.05, c=.2, observations=True)
    ]


def sensitivities():
    # Paired travel settings and finer resolution/smear are explicit joint controls.
    variants = {
        "scan-off": {"use_scan_matching": False},
        "loop-off": {"do_loop_closing": False},
        "travel-fine": {"minimum_travel_distance": .05, "minimum_travel_heading": .05},
        "travel-coarse": {"minimum_travel_distance": .2, "minimum_travel_heading": .2},
        "search-narrow": {"correlation_search_space_dimension": .3},
        "search-wide": {"correlation_search_space_dimension": .8},
        "search-fine-smear03": {"correlation_search_space_resolution": .005,
                               "correlation_search_space_smear_deviation": .03},
        "smear-fine": {"correlation_search_space_smear_deviation": .03},
        "angle-narrow": {"coarse_search_angle_offset": .0698},
        "link-response": {"link_match_minimum_response_fine": .3},
        "link-distance": {"link_scan_maximum_distance": .8},
        "buffer-20": {"scan_buffer_size": 20},
        "buffer-distance": {"scan_buffer_maximum_scan_distance": 3.},
        "response-expansion-off": {"use_response_expansion": False},
        "distance-penalty": {"distance_variance_penalty": .1},
        "angle-penalty": {"angle_variance_penalty": .2},
        "barycenter-off": {"use_scan_barycenter": False},
    }
    return [dict(name=name, x=1., k=-1., c=0., parameters=parameters,
                 observations=True) for name, parameters in variants.items()]


def base_pose(sensor, offset):
    x, y, angle = sensor
    dx, dy, da = offset
    angle -= da
    c, s = np.cos(angle), np.sin(angle)
    return np.array([x-c*dx+s*dy, y-s*dx-c*dy, angle])


def rejected_candidates(directory, dataset, gate):
    """Classify actual rejected proposals, distinct from absent baseline edge pairs."""
    inputs = list(replay.records(directory / "result-inputs.jsonl"))
    online = [r for r in replay.records(directory / "result-online.jsonl") if r["accepted"]]
    source = {r["source_id"]: r for r in inputs}
    nodes = {r["id"]: source[r["source_id"]] for r in online}
    truth = cf.dataset_truth(cf.PROJECT, dataset, inputs)
    decisions = list(replay.records(directory / "result-decisions.jsonl"))
    candidate_path = directory / "result-candidates.jsonl"
    candidates = list(replay.records(candidate_path)) if candidate_path.exists() else []
    threshold = gate.get("parameters", {}).get("loop_match_minimum_response_fine", .45)
    candidates = [r for r in candidates if r["fine_response"] >= threshold]
    assert len(candidates) == len(decisions), (len(candidates), len(decisions))
    rejected = []
    for decision, candidate in zip(decisions, candidates):
        a, b = decision["from_scan_id"], decision["to_scan_id"]
        assert b == candidate["scan_id"]
        expected_arc = nodes[b]["arc_m"] - nodes[a]["arc_m"]
        assert abs(decision["odometry_pair_arc_m"] - expected_arc) < 1e-7
        if gate.get("arc") == "pair":
            bound = gate["k"] * expected_arc + gate["c"]
            assert abs(decision["xy_bound_m"] - bound) < 1e-7
            assert decision["accept"] == (decision["correction_m"] <= bound)
        if decision["accept"]:
            continue
        anchor = next(r["pose"] for r in candidate["chain"] if r["scan_id"] == a)
        measured = replay.relative(base_pose(anchor, nodes[a]["laser_offset"]),
                                   base_pose(candidate["fine_sensor_pose"],
                                             nodes[b]["laser_offset"]))
        expected = replay.relative(replay.at(truth, nodes[a]["time"]),
                                   replay.at(truth, nodes[b]["time"]))
        xy = float(np.linalg.norm(measured[:2] - expected[:2]))
        yaw = abs(float(np.degrees(replay.wrap(measured[2] - expected[2]))))
        rejected.append(dict(pair=[a, b], time=nodes[b]["time"],
                             correct=xy <= .10 and yaw <= 5.,
                             xy_error_m=xy, yaw_error_deg=yaw,
                             fine_response=candidate["fine_response"]))
    return rejected


def one(engine, spec, dataset, gate, output):
    row = cf.run_one(engine, cf.PROJECT, spec, dataset, gate, output)
    directory = output / dataset["name"] / gate["name"]
    rejected = rejected_candidates(directory, dataset, gate)
    metrics = json.loads((directory / "map-metrics.json").read_text())
    criteria = geometry.map_criteria(metrics)
    criteria.update(
        normal_p95=metrics["normal_profile_offset_m"]["p95"] <= .05,
        normal_max=metrics["normal_profile_offset_m"]["max"] <= .15,
        normal_thickness=metrics["normal_profile_thickness_m"]["p95"] <= .15)
    row.update(
        rejected_candidates=rejected,
        correct_candidates_rejected=sum(r["correct"] for r in rejected),
        small_wrong_loops=row["wrong_loops"] - row["wrong_loops_gt_1m"],
        normal_p95_m=metrics["normal_profile_offset_m"]["p95"],
        map_criteria=criteria, map_pass=all(criteria.values()))
    (directory / "summary.json").write_text(json.dumps(row, indent=2) + "\n")
    print(f"DRIFT_ROW {dataset['name']} {gate['name']} "
          f"wrong={row['wrong_loops']} rejected_correct={row['correct_candidates_rejected']} "
          f"pose={row['pose_error_max_m']:.6f} map_pass={row['map_pass']}", flush=True)
    return row


def aggregate(rows):
    result = {}
    for name, selected in (
            ("original_12", [r for r in rows if r["dataset"] != "g02-9"]),
            ("attempt_9", [r for r in rows if r["dataset"] == "g02-9"]),
            ("all_13", rows)):
        groups = {}
        for row in selected:
            groups.setdefault(row["gate"]["name"], []).append(row)
        result[name] = [
            dict(gate=gate, datasets=len(group),
                 wrong_gt_1m=sum(r["wrong_loops_gt_1m"] for r in group),
                 small_wrong=sum(r["small_wrong_loops"] for r in group),
                 correct_candidates_rejected=sum(r["correct_candidates_rejected"] for r in group),
                 correct_pair_absent=sum(len(r.get("correct_loops_lost", [])) for r in group),
                 wall_p95=max(r["wall_offset_p95_m"] for r in group),
                 normal_p95=max(r["normal_p95_m"] for r in group),
                 doubled=sum(r["doubled_walls"] for r in group),
                 ghost=sum(r["ghost_components"] for r in group),
                 pose_max=max(r["pose_error_max_m"] for r in group),
                 pose_p95=max(r["pose_error_p95_m"] for r in group),
                 all_maps_pass=all(r["map_pass"] for r in group))
            for gate, group in groups.items()]
    return result


def write_results(rows, output, filename):
    """Compare original source pairs even when movement filtering renumbers nodes."""
    pairs = {}
    for row in rows:
        baseline_directory = output / row["dataset"] / "none"
        baseline = json.loads((baseline_directory / "summary.json").read_text())
        current_directory = output / row["dataset"] / row["gate"]["name"]
        for directory, item in ((baseline_directory, baseline), (current_directory, row)):
            if directory not in pairs:
                source = {r["id"]: r["source_id"] for r in
                          replay.records(directory / "result-online.jsonl") if r["accepted"]}
                pairs[directory] = [
                    dict(loop, pair=[source[node] for node in loop["pair"]])
                    for loop in item["loops"]]
        row["correct_loops_lost"] = cf.lost_correct(
            pairs[baseline_directory], pairs[current_directory])
    result = dict(aggregate=aggregate(rows), rows=rows)
    (output / filename).write_text(json.dumps(result, indent=2) + "\n")


def diagnose(output, directory):
    """Time-align odometry/SLAM/qpos and join route labels without final-pose fitting."""
    dataset = next(d for d in manifest()["datasets"] if d["name"] == "g02-9")
    inputs = list(replay.records(directory / "result-inputs.jsonl"))
    truth = cf.dataset_truth(cf.PROJECT, dataset, inputs)
    online = [r for r in replay.records(directory / "result-online.jsonl") if r["accepted"]]
    source = {r["source_id"]: r for r in inputs}
    stamps = np.array([r["time"] for r in online])
    odometry = np.array([source[r["source_id"]]["odom"] for r in online])
    poses = np.array([r["pose"] for r in online])
    expected = np.array([replay.at(truth, t) for t in stamps])
    odom_error = np.linalg.norm(odometry[:, :2] - expected[:, :2], axis=1)
    slam_error = np.linalg.norm(poses[:, :2] - expected[:, :2], axis=1)
    root = cf.PROJECT / ".omo/evidence/loop-gate-runtime/g02-attempt9"
    trace = json.loads((root / "mapping-trace.json").read_text())
    trace_times = np.array([r["time"] for r in trace])
    indices = np.maximum(np.searchsorted(trace_times, stamps, side="right") - 1, 0)
    labels = [(trace[i]["target"]["kind"], trace[i]["target"]["wall"]) for i in indices]
    segments, start = [], 0
    for end in range(1, len(labels) + 1):
        if end != len(labels) and labels[end] == labels[start]:
            continue
        selected = slice(start, end)
        segments.append(dict(
            kind=labels[start][0], wall=labels[start][1],
            start_s=float(stamps[start]), end_s=float(stamps[end-1]), scans=end-start,
            odom_max_m=float(odom_error[selected].max()),
            slam_start_m=float(slam_error[start]), slam_end_m=float(slam_error[end-1]),
            slam_max_m=float(slam_error[selected].max()),
            slam_p95_m=float(np.percentile(slam_error[selected], 95)),
            error_growth_m=float(slam_error[end-1] - slam_error[start])))
        start = end
    top = sorted(segments, key=lambda r: r["error_growth_m"], reverse=True)[:15]
    scans = {r["scan_id"]: r for r in replay.records(root / "karto-scans.jsonl")}
    support = {}
    for segment in top:
        selected = np.flatnonzero(
            (stamps >= segment["start_s"]) & (stamps <= segment["end_s"]))
        measurements = []
        for index in selected:
            scan_id = online[index]["source_id"]
            if scan_id not in support:
                support[scan_id] = replay.line_support(scans[scan_id])
            measurements.append(support[scan_id])
        fractions = [r["dominant_line_fraction"] for r in measurements]
        segment.update(
            start_xy=expected[selected[0], :2].tolist(),
            end_xy=expected[selected[-1], :2].tolist(),
            dominant_line_fraction_median=float(np.median(fractions)),
            dominant_line_fraction_max=max(fractions),
            minimum_range_m=min(r["min_range_m"] for r in measurements),
            near_wall_degenerate_scans=sum(r["near_wall_degenerate"] for r in measurements))
    snapshots = []
    for target in (447., 746.596, 756.192, 781.876):
        index = int(np.argmin(np.abs(stamps - target)))
        snapshots.append(dict(time_s=float(stamps[index]), xy=expected[index, :2].tolist(),
                              odom_error_m=float(odom_error[index]),
                              slam_error_m=float(slam_error[index])))
    result = dict(
        odom_max_m=float(odom_error.max()), odom_p95_m=float(np.percentile(odom_error, 95)),
        slam_max_m=float(slam_error.max()), slam_p95_m=float(np.percentile(slam_error, 95)),
        max_time_s=float(stamps[slam_error.argmax()]), segments=segments,
        largest_growth_segments=top, snapshots=snapshots)
    (output / "drift-segments.json").write_text(json.dumps(result, indent=2) + "\n")
    print("DRIFT_SEGMENTS " + json.dumps({k: v for k, v in result.items() if k != "segments"}),
          flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--resume", action="store_true")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--compare", action="store_true")
    mode.add_argument("--sensitivity", action="store_true")
    mode.add_argument("--diagnose", action="store_true")
    mode.add_argument("--profiles", action="store_true")
    mode.add_argument("--summarize", choices=("comparison", "sensitivity"))
    args = parser.parse_args()
    spec = manifest()
    cf.checked_inputs(cf.PROJECT, spec)
    if args.summarize:
        filename = args.summarize + ".json"
        result = json.loads((args.output / filename).read_text())
        write_results(result["rows"], args.output, filename)
        print("DRIFT_SUMMARIZED " + str(args.output / filename), flush=True)
        return
    if args.diagnose:
        diagnose(args.output, args.output / "g02-9/distance-1")
        return
    if args.profiles:
        dataset = next(d for d in spec["datasets"] if d["name"] == "g02-9")
        scans = args.output / "g02-9/distance-1/result-scans.jsonl"
        for name in ("distance-1", "scan-off", "loop-off"):
            result = profiles.diagnose(
                args.output / "g02-9" / name, cf.PROJECT / spec["world"],
                scans_path=scans, transform=dataset["world_to_map"],
                require_no_loops=name != "distance-1")
            (args.output / f"profiles-{name}.json").write_text(
                json.dumps(result, indent=2) + "\n")
            print(f"DRIFT_PROFILE {name} "
                  f"doubled={result['map_metrics']['doubled_wall_components']}", flush=True)
        return
    args.output.mkdir(parents=True, exist_ok=True)
    datasets = spec["datasets"] if args.compare else [
        d for d in spec["datasets"] if d["name"] == "g02-9"]
    selected = gates() if args.compare else sensitivities()
    rows = []
    # Complete none first, then calculate each changed graph's absent baseline pairs.
    waves = (selected[:1], selected[1:]) if args.compare else (selected,)
    for wave in waves:
        work = [(dataset, gate) for dataset in datasets for gate in wave]
        if args.resume:
            pending = []
            for dataset, gate in work:
                summary = args.output / dataset["name"] / gate["name"] / "summary.json"
                if summary.is_file():
                    row = json.loads(summary.read_text())
                    if (row["dataset"] != dataset["name"] or row["gate"] != gate
                            or "map_pass" not in row):
                        raise ValueError(f"incompatible checkpoint: {summary}")
                    rows.append(row)
                    print(f"DRIFT_REUSED {dataset['name']} {gate['name']}", flush=True)
                else:
                    pending.append((dataset, gate))
            work = pending
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            rows.extend(pool.map(lambda item: one(
                args.engine, spec, *item, args.output), work))
    filename = "comparison.json" if args.compare else "sensitivity.json"
    write_results(rows, args.output, filename)
    if args.compare:
        cf.reproduction(next(r for r in rows if r["dataset"] == "g02-6"
                             and r["gate"]["name"] == "none"))
        diagnose(args.output, args.output / "g02-9/distance-1")
    print("DRIFT_COMPLETE " + str(args.output / filename), flush=True)


if __name__ == "__main__":
    main()
