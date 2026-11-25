"""Actual immutable factors + measured matcher scores; replay-only chain sizes.

Replay details are admitted only through the prefix where every actual inserted
loop pair and candidate pose agrees. Missing historical data remains explicitly null.
The grid reports retained correct loops, not a guarantee on unseen scans/robots.
"""
import csv
import itertools
import json
import math
import os
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parent
loops=json.loads((ROOT/"analysis-6/loop-edges.json").read_text())
prefix=os.environ.get("G02_REPLAY_PREFIX","replay-baseline")
replay=ROOT/f"{prefix}-edges.jsonl"
candidates=ROOT/f"{prefix}-candidates.jsonl"
recovered={}
replay_audit={}
if replay.exists() and candidates.exists():
    edges=[json.loads(line) for line in replay.read_text().splitlines()]
    inserted=[row for row in edges if row["loop"] and row["new_edge"]]
    expected=[row["native"] for row in loops]
    actual_pairs=[(row["from_scan_id"],row["to_scan_id"]) for row in expected]
    replay_pairs=[(row["from_scan_id"],row["to_scan_id"]) for row in inserted]
    exact_prefix=[]
    for original,copy in zip(expected,inserted):
        if (original["from_scan_id"],original["to_scan_id"])!=(
                copy["from_scan_id"],copy["to_scan_id"]):
            break
        if max(abs(a-b) for a,b in zip(
                original["match_sensor_pose"],copy["match_sensor_pose"]))>1e-6:
            break
        exact_prefix.append(original)
    replay_audit=dict(expected_pairs=actual_pairs,replay_pairs=replay_pairs,
                      matching_prefix_count=len(exact_prefix),
                      all_15_match=len(exact_prefix)==15 and len(inserted)==15,
                      source_prefix=prefix,
                      recorded_query_hints_restored=prefix=="replay-restored")
    all_candidates=[json.loads(line) for line in candidates.read_text().splitlines()]
    for native in exact_prefix:
        matches=[candidate for candidate in all_candidates
                 if candidate["scan_id"]==native["to_scan_id"] and
                 max(abs(a-b) for a,b in zip(candidate["fine_sensor_pose"],
                                            native["match_sensor_pose"]))<=1e-6]
        if len(matches)!=1:
            raise RuntimeError(f"candidate association is not unique: {native}")
        recovered[(native["from_scan_id"],native["to_scan_id"])]=matches[0]
rows=[]
for loop in loops:
    native,factor,acceptance=loop["native"],loop["factor"],loop["acceptance"]
    coarse=re.fullmatch(
        r"COARSE RESPONSE: ([\d.e+-]+) var: ([\d.e+-]+), ([\d.e+-]+)",
        acceptance["coarse"])
    fine=re.fullmatch(r"FINE RESPONSE: ([\d.e+-]+)",acceptance["fine"])
    a,b=factor["from_pose_at_link"],factor["to_pose_at_link"]
    row=dict(from_id=native["from_scan_id"],to_id=native["to_scan_id"],
             time=native["to_time"],correct=factor["correct"],
             coarse=float(coarse[1]),fine=float(fine[1]),
             coarse_var_x_m2=float(coarse[2]),coarse_var_y_m2=float(coarse[3]),
             fine_covariance_link_frame=factor["covariance"],
             chain_size=None,chain_provenance="not recorded in original run",
             gt_node_distance_m=factor["gt_separation_m"],
             estimated_node_distance_at_link_m=math.hypot(b[0]-a[0],b[1]-a[1]),
             estimated_node_distance_before_match_m=None,
             relative_xy_error_m=factor["relative_xy_error_m"],
             relative_yaw_error_deg=factor["relative_yaw_error_deg"])
    candidate=recovered.get((row["from_id"],row["to_id"]))
    if candidate:
        chain_node=next(item for item in candidate["chain"] if item["scan_id"]==row["from_id"])
        prior=candidate["prior_sensor_pose"]
        row.update(chain_size=candidate["chain_size"],
                   chain_provenance=f"{prefix}; all pairs and poses through this event agree <=1e-6",
                   estimated_node_distance_before_match_m=math.hypot(
                       prior[0]-chain_node["pose"][0],prior[1]-chain_node["pose"][1]),
                   coarse_covariance=candidate["coarse_covariance"])
    rows.append(row)
grid=[]
for coarse,fine,sigma,chain_min in itertools.product(
        [.35,.375,.4,.425,.45,.475,.5],
        [.45,.475,.5,.525,.55,.575,.6],
        [.12,.15,.18,.19,.195,.2,.205,.21,.22,.24,.3,.5,3.],
        [10,12,15,20,25,30,40,50] if len(recovered)==15 else [10]):
    kept=[row for row in rows if row["coarse"]>coarse and row["fine"]>=fine
          and max(row["coarse_var_x_m2"],row["coarse_var_y_m2"])<sigma*sigma
          and (len(recovered)!=15 or row["chain_size"]>=chain_min)]
    if not any(not row["correct"] for row in kept):
        grid.append(dict(coarse=coarse,fine=fine,sigma_parameter_m=sigma,
                         internal_variance_limit_m2=sigma*sigma,
                         minimum_chain_size=chain_min,
                         correct_kept=len(kept),correct_lost=10-len(kept)))
grid.sort(key=lambda row:(-row["correct_kept"],row["coarse"],row["fine"]))
output=ROOT/"analysis-6/threshold-separation"
output.with_suffix(".json").write_text(json.dumps(dict(
    units="coarse covariance diagonal m²; SDK squares the ROS variance parameter",
    chain_recovered=len(recovered)==15,chain_recovered_count=len(recovered),
    replay_audit=replay_audit,rows=rows,separating_candidates=grid),indent=2))
fields=[key for key in rows[0] if key not in ("fine_covariance_link_frame","coarse_covariance")]
with output.with_suffix(".csv").open("w") as stream:
    writer=csv.DictWriter(stream,fieldnames=fields,extrasaction="ignore")
    writer.writeheader()
    writer.writerows(rows)
lines=[
    "G02 baseline 실제 삽입 loop 15개를 immutable 상대 제약과 GT로 판정한 표이다.",
    "",
    "Coarse var는 matcher가 비교하는 m²이며 ROS 파라미터 3.0은 SDK setter에서 "
    "제곱되어 내부 한계 9.0 m²가 된다. Fine covariance 전체 행렬은 JSON에 보존한다.",
    "추정 거리는 연결 당시 sensor origin 거리다. 실제 link 거리 gate는 "
    "scan barycenter를 사용하므로 두 수치를 동일시하지 않는다.",
    "",
    "| edge | sim s | 판정 | coarse | fine | coarse var x/y m² | chain | GT m | 추정 link m | 추정 prior m |",
    "|---|---:|---|---:|---:|---|---:|---:|---:|---:|",
]
for row in rows:
    prior=row["estimated_node_distance_before_match_m"]
    lines.append(
        f"| {row['from_id']} → {row['to_id']} | {row['time']:.3f} | "
        f"{'정상' if row['correct'] else '오류'} | {row['coarse']:.6f} | "
        f"{row['fine']:.6f} | {row['coarse_var_x_m2']:.6g} / "
        f"{row['coarse_var_y_m2']:.6g} | "
        f"{row['chain_size'] if row['chain_size'] is not None else '미기록'} | "
        f"{row['gt_node_distance_m']:.4f} | "
        f"{row['estimated_node_distance_at_link_m']:.4f} | "
        f"{f'{prior:.4f}' if prior is not None else '미기록'} |")
lines.extend(["", f"chain 복구: {len(recovered)}/15개. 원래 edge ID와 candidate pose가 "
    "이 event까지 재생에서 1e-6 이내 일치한 prefix만 사용한다. "
    "미기록 표시는 복구되지 않은 값이며 추정하지 않는다.",
    "재생 입력: "+prefix+". 실제 query hint 복원 모드는 원래 기록된 sensor pose를 "
    "복원하며 GT나 수락 결정을 주입하지 않는다.",
    "", "후보 임계값은 이 자료의 분리를 뜻하며 미관측 환경의 정확성을 보증하지 않는다.",
    "", "가장 많은 정상 loop를 보존하는 후보 10개:", ""])
lines.extend("- "+json.dumps(row,ensure_ascii=False) for row in grid[:10])
output.with_suffix(".md").write_text("\n".join(lines)+"\n")
print("THRESHOLD_TABLE "+json.dumps(dict(rows=len(rows),
    chain_recovered_count=len(recovered),best=grid[:5])),flush=True)
