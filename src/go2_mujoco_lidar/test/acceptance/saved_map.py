"""G03 fixed saved-map coordinates and offline independent telemetry judgement.

No ROS imports: neighbouring pytest checks the same transforms, interpolation and
physics interval reduction used by acceptance.py. Missing samples fail closed.
AMCL errors compare equal-stamp GT in the fixed map frame, never odom coordinates.
"""

import bisect
import csv
import math


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


def command_ownership(log, samples, start_ns, end_ns):
    """Resolve each native RMW handle, then compare half-open 250 ms windows.

    Graph endpoint counts remain evidence, not a single-endpoint requirement.
    Unknown senders and empty observations fail. Receipt time is steady/wall
    time, like the FIFO watchdog; simulation time only selects the drive phase.
    """
    endpoints = {(e["topic"], e["gid"]): e["node"]
                 for sample in samples for e in sample["cmd_vel"]}
    gids, messages = {}, []
    for line in log.splitlines():
        fields = line.split("\t")
        if fields[0] == "GID_MAP":
            gids[fields[1]] = fields[2]
        elif fields[0] == "COMMAND_OWNER" and start_ns <= int(fields[2]) <= end_ns:
            messages.append({
                "received_steady_ns": int(fields[1]), "sim_stamp_ns": int(fields[2]),
                "topic": fields[3], "rmw_gid": fields[4],
                **dict(zip(("vx", "wz", "vy", "vz", "wx", "wy"), map(float, fields[5:]))),
            })
    latest, overlaps = {}, []
    for row in sorted(messages, key=lambda r: r["received_steady_ns"]):
        row["publisher_gid"] = gids.get(row["rmw_gid"])
        row["node"] = endpoints.get((row["topic"], row["publisher_gid"]))
        if row["node"] in NAV2_COMMAND_NODES:
            for gid, previous in latest.items():
                if gid != row["publisher_gid"] and (
                        row["received_steady_ns"] - previous["received_steady_ns"] < 250000000):
                    overlaps.append({"earlier": previous, "later": row})
            latest[row["publisher_gid"]] = row
    external = [{"topic": topic, "gid": gid, "node": node}
                for (topic, gid), node in endpoints.items() if node not in COMMAND_NODES]
    unknown = sum(row["node"] not in COMMAND_NODES for row in messages)
    return {"method": "rclcpp/RMW publisher_gid + Cyclone GID_MAP",
            "window_wall_sec": .25, "messages": messages, "overlaps": overlaps,
            "external_endpoints": external, "unidentified_or_external_messages": unknown,
            "endpoint_counts": [len(s["cmd_vel"]) for s in samples],
            "passed": bool(samples) and bool(messages) and not external and not unknown
                      and not overlaps}
