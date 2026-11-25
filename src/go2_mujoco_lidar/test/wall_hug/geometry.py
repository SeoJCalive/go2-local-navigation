"""Maintained G02 GT occupancy comparison; MJCF/map YAML/transform to overlay and map-metrics.json; promoted from commit 123f8b7 evidence.

Inputs and outputs follow the functions below; see README.md in this directory
and docs/stabilization-2026-11/G02_WALL_HUG.md for provenance and limitations.
Simulation/offline diagnostics only; real robot safety is unconfirmed.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from PIL import Image
import yaml


THRESHOLDS = {
    "wall_offset_p95_m": .05, "wall_offset_max_m": .15,
    "doubled_wall_components": 0, "ghost_wall_components": 0,
    "wall_thickness_p95_m": .15, "slam_pose_max_xy_m": .10,
    "nonfoot_max_depth_m": .015, "nonfoot_sustained_depth_m": .005,
    "nonfoot_max_deep_duration_s": .050, "nonfoot_max_duration_s": 1.,
    "nav2_goal_xy_m": .30,
    "planned_body_radius_m": .28, "planned_open_wall_clearance_m": .10,
}


def obstacles(world):
    return [dict(g.attrib) for g in ET.parse(world).findall("./worldbody/geom")
            if g.get("type") in ("box", "cylinder")]


def distance(points, geoms):
    points = np.asarray(points)
    best = np.full(points.shape[:-1], np.inf)
    for g in geoms:
        x,y,_ = map(float, g["pos"].split())
        size = list(map(float, g["size"].split()))
        d = points - (x,y)
        if g["type"] == "cylinder":
            value = np.linalg.norm(d,axis=-1)-size[0]
        else:
            a = float(g.get("euler","0 0 0").split()[2])
            xx = d[...,0]*math.cos(a)+d[...,1]*math.sin(a)
            yy = -d[...,0]*math.sin(a)+d[...,1]*math.cos(a)
            q = np.stack((np.abs(xx)-size[0], np.abs(yy)-size[1]),axis=-1)
            value = np.linalg.norm(np.maximum(q,0),axis=-1)+np.minimum(np.max(q,axis=-1),0)
        best = np.minimum(best,value)
    return best


def evaluate(world, map_yaml, transform, output):
    spec=yaml.safe_load(Path(map_yaml).read_text())
    image=np.array(Image.open(Path(map_yaml).parent/spec["image"]))
    occupied=image<80
    resolution=spec["resolution"]
    ox,oy,_=spec["origin"]
    rr,cc=np.indices(image.shape)
    mx=ox+(cc+.5)*resolution
    my=oy+(image.shape[0]-rr-.5)*resolution
    yaw,tx,ty=transform
    wx=math.cos(yaw)*(mx-tx)+math.sin(yaw)*(my-ty)
    wy=-math.sin(yaw)*(mx-tx)+math.cos(yaw)*(my-ty)
    signed=distance(np.stack((wx,wy),axis=-1),obstacles(world))
    gt=signed<=0
    # Occupied cells inside the physical wall are correct, not offset by the
    # wall half-thickness. Outside cells use exact distance to the solid.
    offset=np.maximum(signed[occupied],0)
    if not len(offset):
        raise ValueError("map has no occupied wall measurements")
    # Ghost/double components must extend >= .20m and lie > .15m from GT.
    ghosts=occupied&(np.abs(signed)>.15)
    labels,n=label(ghosts)
    sizes=np.bincount(labels.ravel())[1:]
    ghost_count=int((sizes>=max(3,round(.20/resolution))).sum())
    thickness=2*distance_transform_edt(occupied)*resolution
    nonzero=thickness[occupied]
    profiles=[]
    doubled=0
    offsets=[]
    widths=[]
    geoms=obstacles(world)
    def pixels(points):
        px=math.cos(yaw)*points[:,0]-math.sin(yaw)*points[:,1]+tx
        py=math.sin(yaw)*points[:,0]+math.cos(yaw)*points[:,1]+ty
        c=np.floor((px-ox)/resolution).astype(int)
        r=image.shape[0]-1-np.floor((py-oy)/resolution).astype(int)
        valid=(r>=0)&(r<image.shape[0])&(c>=0)&(c<image.shape[1])
        values=np.zeros(len(points),dtype=bool)
        values[valid]=occupied[r[valid],c[valid]]
        return values
    for geom in geoms:
        if geom["type"]!="box":
            continue
        x,y,_=map(float,geom["pos"].split())
        sx,sy,_=map(float,geom["size"].split())
        angle=float(geom.get("euler","0 0 0").split()[2])
        ca,sa=math.cos(angle),math.sin(angle)
        # Wall centerline normal profiles exclude end corners by .15m.
        major,minor=(sx,sy) if sx>=sy else (sy,sx)
        if minor>.10:
            continue  # Broad islands are obstacles, not anomalously thick walls.
        direction=np.array([ca,sa]) if sx>=sy else np.array([-sa,ca])
        normal=np.array([-direction[1],direction[0]])
        run=0
        events=0
        measured=0
        for u in np.arange(-major+.15,major-.15,resolution):
            center=np.array([x,y])+u*direction
            others=[g for g in geoms if g["name"]!=geom["name"]]
            if float(distance(center,others))<minor+.35:
                continue  # Intersecting real geometry is not a duplicate wall.
            normal_offsets=np.arange(-minor-.35,minor+.351,resolution/4)
            hits=pixels(center[None,:]+normal_offsets[:,None]*normal)
            # Split occupied intervals; raster nearest-neighbor duplicates collapse.
            indices=np.flatnonzero(hits)
            groups=np.split(indices,np.flatnonzero(np.diff(indices)>1)+1) if len(indices) else []
            groups=[group for group in groups if len(group)>=2]
            near=[group for group in groups if abs(float(np.mean(normal_offsets[group])))<minor+.20]
            if near:
                measured+=1
                for primary in near:
                    center_offset=float(np.mean(normal_offsets[primary]))
                    # A solid partition has two real faces. Do not mislabel its
                    # two observed surfaces as a doubled SLAM wall.
                    width=float(normal_offsets[primary[-1]]-normal_offsets[primary[0]]+resolution/4)
                    spans_wall=(normal_offsets[primary[0]]<0<normal_offsets[primary[-1]]
                                and width>=2*minor-resolution)
                    offsets.append(abs(center_offset) if spans_wall else min(
                        abs(center_offset-minor),abs(center_offset+minor)))
                    widths.append(width)
            sides=[[group for group in near if (
                float(np.mean(normal_offsets[group]))) * sign >= 0 or (
                    normal_offsets[group[0]]<0<normal_offsets[group[-1]])] for sign in (-1,1)]
            has_double=any(len(side)>1 and max(
                float(np.mean(normal_offsets[p])) for p in side)-min(
                float(np.mean(normal_offsets[p])) for p in side)>.10 for side in sides)
            if has_double:
                run+=1
            else:
                if run*resolution>=.20:
                    events+=1
                run=0
        if run*resolution>=.20:
            events+=1
        doubled+=events
        profiles.append({"geom":geom["name"],"normal_profiles":measured,
                         "doubled_runs_ge_020m":events})
    rendered=np.full((*image.shape,3),245,dtype=np.uint8)
    rendered[image==205]=180
    rendered[gt]=[20,90,255]
    rendered[occupied]=[240,55,35]
    rendered[occupied&gt]=[160,35,175]
    Image.fromarray(rendered).save(output)
    metrics={"wall_offset_m":{"median":float(np.median(offset)),"p95":float(np.percentile(offset,95)),
                              "max":float(offset.max())},
             "ghost_wall_components":ghost_count,
             "doubled_wall_components":doubled,
             "normal_profile_offset_m":{
                 "median":float(np.median(offsets)) if offsets else None,
                 "p95":float(np.percentile(offsets,95)) if offsets else None,
                 "max":float(max(offsets)) if offsets else None},
             "normal_profile_thickness_m":{
                 "p95":float(np.percentile(widths,95)) if widths else None,
                 "max":float(max(widths)) if widths else None},
             "wall_profiles":profiles,
             "wall_thickness_m":{"p95":float(np.percentile(nonzero,95)),"max":float(nonzero.max())},
             "occupied_cells":int(occupied.sum()),"resolution_m":resolution,
             "alignment":{"yaw":yaw,"tx":tx,"ty":ty},
             "thresholds":THRESHOLDS}
    return metrics



def map_criteria(metrics):
    """Apply the recorded map gates; missing geometry is a failed measurement."""
    return {
        "wall_offset_p95": metrics["wall_offset_m"]["p95"] <= THRESHOLDS["wall_offset_p95_m"],
        "wall_offset_max": metrics["wall_offset_m"]["max"] <= THRESHOLDS["wall_offset_max_m"],
        "doubled_walls": metrics["doubled_wall_components"] == 0,
        "ghost_walls": metrics["ghost_wall_components"] == 0,
        "wall_thickness": metrics["wall_thickness_m"]["p95"] <= THRESHOLDS["wall_thickness_p95_m"],
    }


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--transform", nargs=3, type=float, default=[0,0,0])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    metrics = evaluate(args.world, args.map, args.transform, args.output/"gt-overlay.png")
    metrics["criteria"] = map_criteria(metrics)
    metrics["passed"] = all(metrics["criteria"].values())
    (args.output/"map-metrics.json").write_text(json.dumps(metrics, indent=2)+"\n")
    print(json.dumps(metrics))
