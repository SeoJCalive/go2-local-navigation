"""G03 fixed saved-map coordinates and offline independent telemetry judgement.

No ROS imports: neighbouring pytest checks the same transforms, interpolation and
physics interval reduction used by acceptance.py. Missing samples fail closed.
AMCL errors compare equal-stamp GT in the fixed map frame, never odom coordinates.
"""

import bisect
import csv
import math


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
