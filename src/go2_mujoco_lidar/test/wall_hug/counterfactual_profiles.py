"""Diagnose existing doubled-wall metrics without changing their definition.

Read a counterfactual baseline's map, raw observation scans and original odometry.
Reproduce geometry.py's exact per-wall normal profiles, then compare raw endpoints
placed by sequential scan matching versus odometry at each doubled run. JSON keeps
the intervals, timestamps, pose errors and histograms needed to inspect causes.
No matcher, map, test threshold or configuration is modified.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

import geometry
import replay


def endpoints(scans, inputs, vertices):
    corrected, odometry, times, errors = [], [], [], []
    by_id = {row["source_id"]: row for row in inputs}
    final = {row["id"]: row["pose"] for row in vertices}
    # Baselines in the diagnostic cohort have movement_filtered=0 and one laser.
    for scan in scans:
        source = by_id[scan["scan_id"]]
        ranges = np.asarray(scan["ranges"], dtype=float)
        angles = scan["angle_min"] + np.arange(len(ranges)) * scan["angle_increment"]
        valid = np.isfinite(ranges) & (ranges >= scan["range_min"]) & (ranges < scan["range_max"])
        local = np.column_stack((ranges[valid] * np.cos(angles[valid]),
                                 ranges[valid] * np.sin(angles[valid])))
        dx, dy, dyaw = source["laser_offset"]
        def sensor(base):
            x, y, yaw = base
            return (x + math.cos(yaw)*dx - math.sin(yaw)*dy,
                    y + math.sin(yaw)*dx + math.cos(yaw)*dy, yaw + dyaw)
        odom_sensor = sensor(source["odom"])
        final_sensor = sensor(final[scan["scan_id"]])
        for target, pose in ((corrected, final_sensor), (odometry, odom_sensor)):
            x, y, yaw = pose
            rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                                 [math.sin(yaw), math.cos(yaw)]])
            target.append(local @ rotation.T + [x, y])
        times.append(np.full(len(local), scan["time"]))
        errors.append(np.full(len(local), np.linalg.norm(
            np.asarray(final_sensor[:2]) - odom_sensor[:2])))
    return np.concatenate(corrected), np.concatenate(odometry), np.concatenate(times), np.concatenate(errors)


def diagnose(directory, world):
    spec = yaml.safe_load((directory / "result.yaml").read_text())
    occupied = np.asarray(Image.open(directory / spec["image"])) < 80
    resolution = spec["resolution"]
    ox, oy, _ = spec["origin"]
    geoms = geometry.obstacles(world)
    current, odom, times, errors = endpoints(
        replay.records(directory / "result-scans.jsonl"),
        replay.records(directory / "result-inputs.jsonl"),
        replay.records(directory / "result-vertices.jsonl"))
    edges = list(replay.records(directory / "result-edges.jsonl"))
    assert not any(e["loop"] and e["new_edge"] for e in edges), (
        "select a no-loop baseline to isolate sequential scan matching")
    online = list(replay.records(directory / "result-online.jsonl"))
    assert all(row["accepted"] for row in online), "diagnostic cohort must not filter scans"
    profiles = []
    for geom in geoms:
        if geom["type"] != "box":
            continue
        x, y, _ = map(float, geom["pos"].split())
        sx, sy, _ = map(float, geom["size"].split())
        major, minor = (sx, sy) if sx >= sy else (sy, sx)
        if minor > .10:
            continue
        angle = float(geom.get("euler", "0 0 0").split()[2])
        direction = np.array([math.cos(angle), math.sin(angle)]) if sx >= sy else np.array(
            [-math.sin(angle), math.cos(angle)])
        normal = np.array([-direction[1], direction[0]])
        offsets = np.arange(-minor-.35, minor+.351, resolution/4)
        rows, active, runs = [], [], []
        others = [g for g in geoms if g["name"] != geom["name"]]
        for u in np.arange(-major+.15, major-.15, resolution):
            center = np.array([x, y]) + u*direction
            if float(geometry.distance(center, others)) < minor+.35:
                continue
            points = center + offsets[:, None]*normal
            columns = np.floor((points[:, 0]-ox)/resolution).astype(int)
            image_rows = occupied.shape[0]-1-np.floor((points[:, 1]-oy)/resolution).astype(int)
            valid = ((columns >= 0) & (columns < occupied.shape[1]) &
                     (image_rows >= 0) & (image_rows < occupied.shape[0]))
            hits = np.zeros(len(points), dtype=bool)
            hits[valid] = occupied[image_rows[valid], columns[valid]]
            indices = np.flatnonzero(hits)
            groups = np.split(indices, np.flatnonzero(np.diff(indices)>1)+1) if len(indices) else []
            groups = [g for g in groups if len(g) >= 2 and abs(float(np.mean(offsets[g]))) < minor+.20]
            sides = [[g for g in groups if float(np.mean(offsets[g]))*sign >= 0 or
                      offsets[g[0]] < 0 < offsets[g[-1]]] for sign in (-1, 1)]
            doubled = any(len(side) > 1 and max(float(np.mean(offsets[g])) for g in side) -
                          min(float(np.mean(offsets[g])) for g in side) > .10 for side in sides)
            row = dict(u_m=float(u), intervals_m=[
                [float(offsets[g[0]]), float(offsets[g[-1]])] for g in groups], doubled=doubled)
            rows.append(row)
            if doubled:
                active.append(row)
            else:
                if len(active)*resolution >= .20:
                    runs.append(active)
                active = []
        if len(active)*resolution >= .20:
            runs.append(active)
        evidence = []
        for run in runs:
            low, high = run[0]["u_m"]-resolution/2, run[-1]["u_m"]+resolution/2
            item = dict(u_interval_m=[low, high], counted_length_m=len(run)*resolution,
                        representative=run[len(run)//2], placements={})
            for name, points in (("scan_matched", current), ("odometry", odom)):
                local = points - [x, y]
                along, across = local @ direction, local @ normal
                selected = (along >= low) & (along <= high) & (np.abs(across) < minor+.35)
                values = across[selected]
                histogram, edges = np.histogram(values, bins=np.arange(-minor-.35, minor+.361, .01))
                # Keep all bins; peak selection alone could conceal a real second face.
                buckets = []
                selected_times = times[selected]
                for positions in np.array_split(np.argsort(selected_times), 4):
                    if len(positions):
                        v = values[positions]
                        buckets.append(dict(time_s=[float(selected_times[positions[0]]),
                                                   float(selected_times[positions[-1]])],
                                            n=len(v), normal_m=[float(np.percentile(v, p))
                                                               for p in (5, 50, 95)]))
                item["placements"][name] = dict(
                    n=len(values), normal_histogram=histogram.tolist(), bin_edges_m=edges.tolist(),
                    normal_quantiles_m=[float(np.percentile(values, p)) for p in (5, 50, 95)]
                    if len(values) else None, time_quarters=buckets,
                    sensor_xy_error_max_m=float(errors[selected].max()) if selected.any() else None)
            evidence.append(item)
        profiles.append(dict(geom=geom["name"], half_thickness_m=minor,
                             doubled_runs_ge_020m=len(runs), runs=evidence))
    expected = json.loads((directory / "map-metrics.json").read_text())
    assert sum(p["doubled_runs_ge_020m"] for p in profiles) == expected["doubled_wall_components"]
    for profile, reference in zip(profiles, expected["wall_profiles"]):
        assert (profile["geom"], profile["doubled_runs_ge_020m"]) == (
            reference["geom"], reference["doubled_runs_ge_020m"])
    return dict(dataset=directory.parent.name, loop_edges=0, map_metrics=expected,
                profiles=profiles)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    for name in ("sep-horizontal", "sep-wallfullclean", "sep-wallboth"):
        result = diagnose(args.runs / name / "none", args.world)
        (args.output / f"{name}.json").write_text(json.dumps(result, indent=2)+"\n")
        print(f"PROFILE_DONE {name} doubled={result['map_metrics']['doubled_wall_components']}",
              flush=True)


if __name__ == "__main__":
    main()
