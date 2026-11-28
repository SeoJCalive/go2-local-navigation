"""G03 fixed saved-map coordinates and offline independent telemetry judgement.

No ROS imports: neighbouring pytest checks the same transforms, interpolation and
physics interval reduction used by acceptance.py. Missing samples fail closed.
AMCL errors compare equal-stamp GT in the fixed map frame, never odom coordinates.
"""

import bisect
from collections import Counter
import csv
import math
import os


COMMAND_TOPICS = ("/go2_nav2/shadow_cmd_vel", "/cmd_vel")
NAV2_COMMAND_NODES = {"/controller_server", "/behavior_server"}
COMMAND_NODES = NAV2_COMMAND_NODES | {"/go2_nav2_mujoco_bridge"}
# H3 frozen/result.json, order yaw, tx, ty; not fitted to this run's AMCL.
WORLD_MAP = (6.918148519170386e-05, -0.0014251051562314404, 4.5958254625307515e-05)
GOALS = [[.8, 0.], [1.5, -1.], [0., 0.]]


def map_pose(x, y, angle=0.):
    a, tx, ty = WORLD_MAP
    return [tx + math.cos(a)*x - math.sin(a)*y,
            ty + math.sin(a)*x + math.cos(a)*y, angle+a]


def amcl_errors(poses, truth):
    """Interpolate qpos XY only between bracketing stamps; retain missing as None."""
    stamps = [float(row["sim_time"]) for row in truth]
    rows = []
    for stamp, x, y, angle in poses:
        i = bisect.bisect_right(stamps, stamp)
        error = None
        if 0 < i < len(stamps):
            left, right = truth[i-1], truth[i]
            ratio = (stamp-stamps[i-1])/(stamps[i]-stamps[i-1])
            gx, gy = [float(left[k])+ratio*(float(right[k])-float(left[k])) for k in ("x", "y")]
            mx, my, _ = map_pose(gx, gy)
            error = math.hypot(x-mx, y-my)
        rows.append({"sim_time": stamp, "x": x, "y": y, "yaw": angle, "xy_error_m": error})
    return rows


def physics_summary(rows, start=-math.inf, end=math.inf):
    rows = [row for row in rows if start <= float(row["sim_time"]) <= end]
    if not rows:
        return {"frames": 0, "min_clearance_m": None, "falls": None, "minimum": None}
    minimum = min(rows, key=lambda row: float(row["min_clearance_m"]))
    return {"frames": len(rows), "min_clearance_m": float(minimum["min_clearance_m"]),
            "falls": sum(int(row["fall"]) for row in rows), "minimum": minimum}


def load_csv(path):
    with path.open() as file:
        return list(csv.DictReader(file))


def truth_tail(path, size=4096):
    """Read numeric GT rows while native appends; discard both cut boundaries.

    A nonzero seek can begin inside a field even when its comma count is valid.
    Only newline-terminated, complete numeric rows may reach driving or stopping.
    """
    with path.open("rb") as file:
        header = file.readline().decode().strip().split(",")
        file.seek(0, os.SEEK_END)
        offset = max(0, file.tell()-size)
        file.seek(offset)
        if offset:
            file.readline()
        tail = file.read().decode()
    rows = []
    for line in tail.splitlines(keepends=True):
        if not line.endswith("\n"):
            continue
        fields = line.strip().split(",")
        if len(fields) != len(header) or fields == header:
            continue
        try:
            row = dict(zip(header, map(float, fields)))
        except ValueError:
            continue
        if all(math.isfinite(value) for value in row.values()):
            rows.append(row)
    return rows


def command_ownership(log, samples, start_ns, end_ns, safety_stop=None):
    """Resolve each native RMW handle, then compare half-open 250 ms windows.

    Graph endpoint counts remain evidence, not a single-endpoint requirement.
    Unknown senders and empty observations fail. Receipt time is steady/wall
    time, like the FIFO watchdog; simulation time only selects the drive phase.
    """
    endpoints = {(e["topic"], e["gid"]): e["node"]
                 for sample in samples for e in sample["cmd_vel"]}
    gids, messages, action_snapshots, action_topics = {}, [], {}, set()
    for line in log.splitlines():
        fields = line.split("\t")
        if fields[0] == "GID_MAP":
            gids[fields[1]] = fields[2]
        elif fields[0] == "ACTION_TOPIC":
            action_topics.add(fields[1])
        elif fields[0] == "ACTION_STATUS":
            action_snapshots.setdefault(fields[2], []).append((
                int(fields[1]), {item.split(":")[0]: int(item.split(":")[1])
                                 for item in fields[3].split(",") if item}))
        elif fields[0] == "COMMAND_OWNER" and start_ns <= int(fields[2]) <= end_ns:
            messages.append({
                "received_steady_ns": int(fields[1]), "sim_stamp_ns": int(fields[2]),
                "topic": fields[3], "rmw_gid": fields[4],
                **dict(zip(("vx", "wz", "vy", "vz", "wx", "wy"), map(float, fields[5:]))),
            })
    intervals = []
    for topic, snapshots in action_snapshots.items():
        active = {}
        for received, statuses in sorted(snapshots):
            for uuid in list(active):
                if statuses.get(uuid) not in (1, 2, 3):
                    intervals.append({"topic": topic, "goal_id": uuid,
                                      "start_ns": active.pop(uuid), "end_ns": received})
            for uuid, status in statuses.items():
                if status in (1, 2, 3):
                    active.setdefault(uuid, received)
        intervals.extend({"topic": topic, "goal_id": uuid,
                          "start_ns": received, "end_ns": None}
                         for uuid, received in active.items())
    stops = [safety_stop] if isinstance(safety_stop, dict) else safety_stop or []
    safety_endpoints = {
        (COMMAND_TOPICS[0], e["gid"]): e["node"]
        for stop in stops for e in stop.get("zero_endpoints", [])}
    latest, overlaps = {}, []
    for row in sorted(messages, key=lambda r: r["received_steady_ns"]):
        row["publisher_gid"] = gids.get(row["rmw_gid"])
        row["node"] = endpoints.get((row["topic"], row["publisher_gid"]))
        row["safety_zero"] = any(
            row["received_steady_ns"] >= stop["request_steady_ns"]
            and any(e["gid"] == row["publisher_gid"] and e["node"] == row["node"]
                    for e in stop.get("zero_endpoints", []))
            and row["topic"] == COMMAND_TOPICS[0]
            and row["node"] is not None
            and all(row[k] == 0 for k in ("vx", "wz", "vy", "vz", "wx", "wy"))
            for stop in stops)
        row["active_actions"] = sorted({
            interval["topic"] for interval in intervals
            if interval["start_ns"] <= row["received_steady_ns"] and (
                interval["end_ns"] is None or row["received_steady_ns"] < interval["end_ns"])})
        row["action_role_matches"] = (
            row["node"] == "/controller_server" and
            "/follow_path/_action/status" in row["active_actions"]) or (
                row["node"] == "/behavior_server" and any(
                    topic in row["active_actions"] for topic in (
                        "/spin/_action/status", "/backup/_action/status",
                        "/drive_on_heading/_action/status", "/wait/_action/status",
                        "/assisted_teleop/_action/status")))
        if row["node"] in NAV2_COMMAND_NODES:
            for gid, previous in latest.items():
                if gid != row["publisher_gid"] and (
                        row["received_steady_ns"] - previous["received_steady_ns"] < 250000000):
                    overlaps.append({"earlier": previous, "later": row})
            latest[row["publisher_gid"]] = row
    external = [{"topic": topic, "gid": gid, "node": node}
                for (topic, gid), node in endpoints.items()
                if node not in COMMAND_NODES and safety_endpoints.get((topic, gid)) != node]
    unknown = sum(row["node"] not in COMMAND_NODES and not row["safety_zero"] for row in messages)
    return {"method": "rclcpp/RMW publisher_gid + Cyclone GID_MAP",
            "window_wall_sec": .25, "messages": messages, "overlaps": overlaps,
            "external_endpoints": external, "unidentified_or_external_messages": unknown,
            "endpoint_counts": [len(s["cmd_vel"]) for s in samples],
            "attribution_counts": dict(Counter(row["node"] or "UNIDENTIFIED" for row in messages)),
            "gid_attribution_counts": dict(Counter(
                row["publisher_gid"] or "UNIDENTIFIED" for row in messages)),
            "fallback": "disabled; unidentified primary sender fails closed",
            "fallback_messages": 0,
            "action_status": {
                "topics_discovered": sorted(action_topics),
                "topics": sorted(action_snapshots), "intervals": intervals,
                "snapshots": sum(map(len, action_snapshots.values())),
                "messages_with_active_action": sum(bool(row["active_actions"]) for row in messages),
                "messages_without_active_action": sum(not row["active_actions"] for row in messages),
                "role_matches": sum(row["action_role_matches"] for row in messages),
                "role_unconfirmed": sum(not row["action_role_matches"] for row in messages),
            },
            "passed": bool(samples) and bool(messages) and not external and not unknown
                      and not overlaps}
