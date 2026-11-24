"""MJCF world geometry against the mapping route's swept circular footprint.

The 0.42 m radius is the unchanged simulation Nav2 footprint. Sample spacing
is at most 0.01 m; subtract half that spacing for a conservative continuous
segment bound (distance to a closed obstacle is 1-Lipschitz). This is a planned
route check; per-step simulator contacts separately verify the actual gait.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def obstacle_distance(point, geom):
    """Signed planar distance to an MJCF box or upright cylinder."""
    x, y, _ = map(float, geom.get("pos", "0 0 0").split())
    size = list(map(float, geom["size"].split()))
    dx, dy = point[0] - x, point[1] - y
    if geom["type"] == "cylinder":
        return math.hypot(dx, dy) - size[0]
    if geom["type"] != "box":
        raise ValueError(f"unsupported world geometry: {geom}")
    angle = float(geom.get("euler", "0 0 0").split()[2])
    ca, sa = math.cos(angle), math.sin(angle)
    qx, qy = abs(ca*dx+sa*dy)-size[0], abs(-sa*dx+ca*dy)-size[1]
    return math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0)


def clearance(world, route, radius=.42):
    geoms = [dict(g.attrib) for g in ET.parse(world).findall("./worldbody/geom")
             if g.get("type") != "plane"]
    segments = []
    for index, (a, b) in enumerate(zip([[0., 0.]] + route, route)):
        length = math.dist(a, b)
        count = max(1, math.ceil(length/.01))
        closest = min(
            (obstacle_distance((a[0]+(b[0]-a[0])*i/count,
                                a[1]+(b[1]-a[1])*i/count), g), g["name"])
            for i in range(count+1) for g in geoms)
        segments.append(dict(index=index, obstacle=closest[1],
                             center_distance=closest[0],
                             footprint_clearance_lower_bound=closest[0]-radius-length/count/2))
    return dict(radius=radius, segments=segments,
                minimum=min(r["footprint_clearance_lower_bound"] for r in segments))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--route", type=Path, required=True)
    args = parser.parse_args()
    result = clearance(args.world, json.loads(args.route.read_text()))
    print(json.dumps(result, indent=2))
    if result["minimum"] <= 0:
        raise SystemExit("route footprint intersects world geometry")
