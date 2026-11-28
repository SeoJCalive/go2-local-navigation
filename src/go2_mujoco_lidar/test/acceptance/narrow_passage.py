"""G04 frozen route and independent offline passage/recovery judgement.

acceptance.py supplies real path, qpos and native status records. This module
has no ROS dependency; neighbouring tests exercise the same verdict functions.
Coordinates and exclusions are owned by the preregistered G04 criteria document.
"""
import csv
import math

from saved_map import WORLD_MAP, map_pose, physics_summary

PI = math.pi
# id, x, y, yaw, intended passage; approaches/returns have no passage claim.
ROUTE = [
    ("C01", .8, 0, 0, ""), ("C02", 1.5, -1, 0, ""), ("C03", 0, 0, 0, ""),
    ("S01", -4.2, 0, -PI/2, "P14"), ("S02", -4.75, -1.6, -PI/2, ""),
    ("S03", -4.75, -3.1, PI, "P01"), ("S04", -6.7, -3.15, -PI/2, "P03"),
    ("S05", -6.713933569, -5.184581478, 0, "P05"),
    ("S06", -4.013933535, -5.184768268, PI/2, "P02"),
    ("S07", -3.913819384, -3.534775166, PI/2, ""),
    ("S08", -3.913791712, -3.134775161, 0, "P04"),
    ("S09", -2.413791693, -3.134878933, 0, "P06"), ("S10", 1.5, -3.1, PI/2, ""),
    ("S11", 1.5, -1.7, PI, "P13"), ("S12", 0, 0, 0, ""),
    ("E01", .536547333, 1.764917043, 0, ""),
    ("E02", 2.436540439, 1.664785597, -PI/2, "P26"),
    ("E03", 2.445, .25, 0, "P27"), ("E04", 4.2, 0, 0, "P17"),
    ("E05", 5.186395192, -.435404679, 0, ""), ("E06", 7.1, -.38, -PI/2, "P38"),
    ("E07", 7.086312198, -1.635536139, PI/2, "P29"), ("E08", 7.04, 1.6, PI/2, "P30"),
    ("E09", 4.2, 0, PI, ""), ("E10", 0, 0, 0, ""),
    ("N01", -.2, 1.7, PI/2, ""), ("N02", -.2, 3.2, PI, "P11"),
    ("N03", -2.5, 3.9, PI, ""), ("N04", -3.813314358, 3.765218007, 0, "P15"),
    ("N05", 0, 0, 0, ""),
    ("X01", -3.2, -2.27, 0, "P07"), ("R01", 0, 0, 0, ""),
    ("X02", -6.2, 2.65, 0, "P08"), ("R02", 0, 0, 0, ""),
    ("X03", -6.2, 3.85, 0, "P09"), ("R03", 0, 0, 0, ""),
    ("X04", 5.9, 2.08, 0, "P10"), ("R04", 0, 0, 0, ""),
    ("B01", 5.286315634, -1.585411611, 0, ""),
    ("B02", 7.036312197, -1.635532680, 0, "P28"),
    ("B03", 3.786696114, 3.914692230, 0, ""), ("B04", 5.1, 2.93, 0, "P25"),
]
GOAL_SPECS = [
    {"id": name, "kind": "negative" if name.startswith("X") else
     "boundary" if name.startswith("B") else "positive",
     "world_goal": [x, y, angle], "map_goal": map_pose(x, y, angle),
     "passage": passage, "time_limit_s": 150}
    for name, x, y, angle, passage in ROUTE
]
THROATS = {
    "P14": ((-3.2, -.8), (-3.2, .8)),
    "P01": ((-5.4, -2.4), (-4.1, -2.4)),
    "P03": ((-5.4, -3.83), (-5.4, -2.46)),
    "P05": ((-7.9, -4.25), (-5.85, -4.25)),
    "P02": ((-5.2, -5.9), (-5.2, -4.67)),
    "P04": ((-4.55, -4.1), (-3.26, -4.1)),
    "P06": ((-3.2, -3.5), (-3.2, -2.46)),
    "P13": ((.8, -2.4), (2.2, -2.4)),
    "P26": ((1.45, 1.35), (1.45, 2.34)),
    "P27": ((1.75, 1.05), (3.14, 1.05)),
    "P17": ((3.2, -.8), (3.2, .8)),
    "P38": ((6.196232279723005, -.9400273008873541), (6.18, .18)),
    "P29": ((6.46, -1.2), (7.9, -1.2)),
    "P30": ((6.18, 1), (7.9, 1)),
    "P11": ((-1.2, 2.4), (.8, 2.4)),
    "P15": ((-3.2, 3.3), (-3.2, 4.5)),
    "P07": ((-3.2, -2.34), (-3.2, -2.2)),
    "P08": ((-6.2, 2.46), (-6.2, 2.84)),
    "P09": ((-6.2, 3.56), (-6.2, 4.14)),
    "P10": ((5.9, 1.82), (5.9, 2.34)),
    "P28": ((6.2, -2.34), (6.2, -1.46)),
    "P25": ((4.5, 2.46), (4.5, 3.4)),
}
SW_ORDER = list(THROATS)[:8]
POSITIVE_PASSAGES = list(THROATS)[:16]
RECOVERY_TOPICS = {f"/{name}/_action/status" for name in ("spin", "backup", "wait")}


def goal_placement(grid):
    """Check actual frozen FREE cells and conservative distance to inscribed squares.

    Negative rejection targets intentionally retain their blocked v1 positions.
    The half-cell diagonal makes centre distance a lower bound for cell regions.
    """
    res, (ox, oy), width = grid["resolution"], grid["origin"], grid["width"]
    inscribed = [(ox+(k % width+.5)*res, oy+(k // width+.5)*res)
                 for k, value in enumerate(grid["data"]) if value == 99]
    checks = []
    for spec in GOAL_SPECS:
        if spec["kind"] == "negative":
            continue
        x, y = spec["map_goal"][:2]
        i, j = math.floor((x-ox)/res), math.floor((y-oy)/res)
        cost = grid["data"][j*width+i] if 0 <= i < width and 0 <= j < grid["height"] else None
        distance = min((math.hypot(x-u, y-v) for u, v in inscribed), default=-math.inf)
        margin = distance-res/math.sqrt(2)
        checks.append({"id": spec["id"], "map_goal": spec["map_goal"],
                       "cell": [i, j], "cost": cost, "inscribed_margin_m": margin,
                       "passed": cost == 0 and margin >= .20})
    return {"goals": checks, "passed": len(checks) == 38 and all(r["passed"] for r in checks)}


def world_xy(point):
    a, tx, ty = WORLD_MAP
    x, y = point[0]-tx, point[1]-ty
    return [math.cos(a)*x+math.sin(a)*y, -math.sin(a)*x+math.cos(a)*y]


def throat_coordinates(point, throat):
    a, b = throat
    vx, vy = b[0]-a[0], b[1]-a[1]
    length = math.hypot(vx, vy)
    px, py = point[0]-a[0], point[1]-a[1]
    return (px*vx+py*vy)/length, (vx*py-vy*px)/length, length


def crossings(points, throat):
    """Segment/finite throat intersections, excluding tangency and endpoints."""
    hits = []
    off_line = [(i, p) for i, p in enumerate(points)
                if abs(throat_coordinates(p, throat)[1]) > 1e-10]
    for (i, left), (j, right) in zip(off_line, off_line[1:]):
        u, s, length = throat_coordinates(left, throat)
        v, t, _ = throat_coordinates(right, throat)
        if s*t < 0:
            fraction = s/(s-t)
            if 0 < u+fraction*(v-u) < length:
                hits.append(i+fraction*(j-i))
    return hits


def entered(points, throat):
    return any(0 < u < length and abs(s) <= .15
               for u, s, length in (throat_coordinates(p, throat) for p in points))


def negative_outcome(row, points):
    path_count = row.get("planned_path_count")
    no_entry = bool(points) and not entered(points, THROATS[row["passage"]]) and not crossings(
        points, THROATS[row["passage"]])
    return {"aborted": row["status"] == 6,
            "within_time": row["wall_sec"] <= 150,
            "no_path": path_count == 0,
            "costmap_blocked": row.get("tolerance_allowed_cells") == 0,
            "no_entry": no_entry}


def verdict_physics(rows, boundary_start):
    return physics_summary([r for r in rows if float(r["sim_time"]) < boundary_start])


def passage_coverage(truth, paths):
    points = [[float(r["x"]), float(r["y"])] for r in truth]
    gt = {p: crossings(points, THROATS[p]) for p in POSITIVE_PASSAGES}
    planned = {p: [] for p in POSITIVE_PASSAGES}
    for path_index, path in enumerate(paths):
        if path.get("kind") != "positive":
            continue
        route = [world_xy(p) for p in path["poses"]]
        for p in POSITIVE_PASSAGES:
            planned[p].extend((path_index, hit) for hit in crossings(route, THROATS[p]))
    gt_order = all(gt[p] for p in SW_ORDER) and all(
        gt[a][0] < gt[b][0] for a, b in zip(SW_ORDER, SW_ORDER[1:]))
    plan_order = all(planned[p] for p in SW_ORDER) and all(
        min(planned[a]) < min(planned[b]) for a, b in zip(SW_ORDER, SW_ORDER[1:]))
    return {"gt": gt, "planned": planned, "sw_gt_order": gt_order,
            "sw_plan_order": plan_order,
            "passed": gt_order and plan_order and all(gt.values()) and all(planned.values())}


def recoveries(log, ownership):
    """Preserve each actual status transition; require observed terminal states."""
    events = {}
    for line in log.splitlines():
        fields = line.split("\t")
        if fields[0] != "ACTION_STATUS" or fields[2] not in RECOVERY_TOPICS:
            continue
        for value in fields[3].split(","):
            if not value:
                continue
            uuid, status = value.split(":")
            key = (fields[2], uuid)
            row = events.setdefault(key, {"topic": fields[2], "goal_id": uuid, "states": []})
            state = int(status)
            if not row["states"] or row["states"][-1]["status"] != state:
                row["states"].append({"steady_ns": int(fields[1]), "status": state})
    result = []
    for row in events.values():
        active = [s for s in row["states"] if s["status"] in (1, 2, 3)]
        if not active:
            continue
        row["start_ns"] = active[0]["steady_ns"]
        terminal = [s for s in row["states"] if s["status"] in (4, 5, 6)]
        row["terminal"] = terminal[-1]["status"] if terminal else None
        row["end_ns"] = terminal[-1]["steady_ns"] if terminal else None
        row["wall_sec"] = (row["end_ns"]-row["start_ns"])/1e9 if terminal else None
        row["senders"] = sorted({
            (m["node"], m["publisher_gid"]) for m in ownership["messages"]
            if row["start_ns"] <= m["received_steady_ns"]
            and (row["end_ns"] is None or m["received_steady_ns"] <= row["end_ns"])
            and m["node"] == "/behavior_server"})
        result.append(row)
    return result


class SafetyReader:
    """Consume new complete native CSV lines only; never rescan old frames."""
    def __init__(self, output):
        self.output = output
        self.offsets = {}
        self.minimum = math.inf
        self.alert_row = None

    def poll(self):
        alert = None
        for name in ("physics", "contacts"):
            path = self.output / f"{name}.csv"
            if not path.exists():
                continue
            offset, header = self.offsets.get(name, (0, None))
            with path.open() as file:
                file.seek(offset)
                if header is None:
                    line = file.readline()
                    if not line.endswith("\n"):
                        continue
                    header = line.strip().split(",")
                while True:
                    position = file.tell()
                    line = file.readline()
                    if not line.endswith("\n"):
                        file.seek(position)
                        break
                    row = next(csv.DictReader([line], fieldnames=header))
                    candidate = None
                    if name == "contacts":
                        candidate = f"비바닥 접촉 sim={row['sim_time']}"
                    else:
                        gap = float(row["min_clearance_m"])
                        self.minimum = min(self.minimum, gap)
                        if int(row["fall"]) or gap < .10:
                            candidate = f"안전 경계 sim={row['sim_time']} fall={row['fall']} clearance={gap}"
                    if candidate and (alert is None or
                                      float(row["sim_time"]) < float(self.alert_row["sim_time"])):
                        alert, self.alert_row = candidate, row
                self.offsets[name] = (file.tell(), header)
        return alert
