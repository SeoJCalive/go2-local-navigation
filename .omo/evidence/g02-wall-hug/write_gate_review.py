"""Publish the requested Korean offline comparison; never changes product config."""
import csv
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/"offline-gate-review"
rows=json.loads((OUT/"all-accepted-edges.json").read_text())
summary=json.loads((OUT/"gate-comparison.json").read_text())
frontier=json.loads((OUT/"exact-config-frontier.json").read_text())
replay=json.loads((OUT/"gui-yaml-full-replay-summary.json").read_text())
def edge(r):
    return f"{r['attempt']}:{r['from_id']}->{r['to_id']}"
def f(x):
    return f"{x:.4f}"
report=[
    "# G02 6·7차 실행: 오프라인 loop gate 비교",
    "",
    "이 문서는 승인할 수정 방향을 고를 때 쓰는 기록이다. 제품 설정·코드의 추가 변경이나 GUI 재실행은 하지 않았다.",
    "",
    "- 실제로 새 삽입된 loop 제약 16개: 6차 15개(오류 5·정상 10), 7차 1개(오류 1). accepted callback 중 기존 edge에 연결한 횟수는 이 수에 포함하지 않는다.",
    "- 정상 판정: 불변 상대 측정과 qpos GT의 XY 차이 <=0.10m, yaw 차이 <=5도. GT 두 노드의 거리 자체가 정상/오류의 판정 기준은 아니다.",
    "- 6차 chain/후보 pose는 기록 hint를 사용하는 동일 dataset 재생에서 15개 실제 edge를 모두 재현한 값이다. 실제 pose와 최대 차이 1.21e-8이며 GT를 재생 입력에 쓰지 않았다.",
    "- 7차 chain/후보 pose/응답/coarse covariance는 실제 native 기록이다. fine LinkInfo covariance와 odometry dataset은 사용자 지시 중단 시 직렬화되지 않아 남아 있지 않다.",
    "- 전체 coarse covariance 9개와 6차 fine LinkInfo covariance 9개는 all-accepted-edges.json에 보존했다. 7차 fine covariance를 추정값으로 채우지 않았다.",
    "",
    "## 실제 적용값과 재생",
    "",
    "- 중단 전 실제 /slam_toolbox 조회: sigma 0.19, coarse 0.35, fine 0.45, chain 10, loop search dimension 8.0.",
    "- GUI의 설치 YAML을 그대로 ROS parameter loader와 SMapper::configure에 넣은 전체 baseline 재생: 2,099 scan 수락, movement 필터 탈락 0.",
    "- 실제 Karto 출력: dimension=8, sigma=0.19, internal variance=0.0361m², coarse=.35, fine=.45, chain=10, candidate distance=3m.",
    f"- 이 전체 재생은 140->275를 삽입하지 않았지만 새 loop {replay['inserted_loops']}개 중 GT 정상 {replay['correct']}개·오류 {replay['wrong']}개를 삽입했다. 따라서 설정 경로 누락이 아니라 다른 confident alias가 생기는 문제다.",
    "- 재생 전체 결과는 gui-yaml-full-replay-summary.json에 있다. 아래 gate별 수치는 두 실제 실행의 고정된 수락 후보 16개를 순서대로 다시 판정한 결과다. gate를 바꿔 그래프를 다시 최적화한 후의 모든 새 후보에 대한 성공 증명으로 해석하지 않는다.",
    "",
    "## 모든 실제 삽입 제약",
    "",
    "coarse Vx/Vy는 Karto 내부 m² 단위다. Δ는 매칭 직전 query pose에서 fine pose까지의 XY 보정 크기다. D는 직전 실제 삽입 loop 이후 이동 거리다. 첫 loop의 기준은 첫 scan이다.",
    "",
    "| 실행:edge | GT 판정 | coarse/fine | coarse Vx/Vy m² | chain | GT 노드 거리 m | Δ m | D m |",
    "|---|---|---|---|---:|---:|---:|---:|",
]
for r in rows:
    report.append(f"| {edge(r)} | {'정상' if r['correct'] else '오류'} | "
        f"{f(r['coarse'])}/{f(r['fine'])} | "
        f"{f(r['coarse_covariance'][0])}/{f(r['coarse_covariance'][4])} | "
        f"{r['chain_size']} | {f(r['gt_node_distance_m'])} | "
        f"{f(r['implied_correction_m'])} | {f(r['distance_since_actual_loop_m'])} |")
report.extend([
    "",
    "- 6차 D는 저장된 실제 scan odometry에서 구했다. scan 표본 누적 거리는 272.148744m이며, 같은 시각 qpos 누적 거리는 272.149574m다.",
    "- 7차 D=35.946018m는 실제 scan 시각의 qpos 대체 측정이다. odometry 원본을 기록하지 않았으므로 실제 odometry 이동 거리라고 주장하지 않는다.",
    "- 새 오류 148->284는 sim 202.344s, GT 노드 거리 2.061753m, 상대 XY 측정 오차 3.545454m, yaw 오차 0.651753도, 보정 3.500014m다.",
    "",
    "## 양쪽 scan 형상",
    "",
    "직선 지지율은 센서 좌표계의 유효 endpoint 중 한 직선에서 0.03m 이내인 비율이다. 결정적인 2,048 pair 가설을 사용했다. 퇴화 기준은 지지율 >=70%이면서 min range <=0.50m다. GT나 지도 없이 계산했다.",
    "",
    "| 실행:edge | source/query 직선 지지율 % | source/query min range m | 퇴화 source/query |",
    "|---|---|---|---|",
])
for r in rows:
    a,b=r["from_geometry"]["sensor_only"],r["to_geometry"]["sensor_only"]
    report.append(f"| {edge(r)} | {a['dominant_line_fraction']*100:.2f}/"
        f"{b['dominant_line_fraction']*100:.2f} | {a['min_range_m']:.3f}/"
        f"{b['min_range_m']:.3f} | "
        f"{'예' if a['near_wall_degenerate'] else '아니오'}/"
        f"{'예' if b['near_wall_degenerate'] else '아니오'} |")
report.extend([
    "",
    "- 별도의 GT 벽 endpoint 대응에서도 두 실행의 모든 node가 70% 단일 벽 기준에 못 미쳤다. 가까운 벽을 본다는 사실만으로 전체 scan이 직선으로 퇴화했다고 판정할 근거는 없다.",
    "- 두 최초 오연결의 min range는 0.25m blind zone보다 크다. 첫 오연결 원인을 min range 절단으로 확정할 근거는 없다.",
    "",
    "## gate 비교",
    "",
    "아래 분모는 오류 6·정상 10이다. 나머지 파라미터는 원래 coarse .35/fine .45/chain 10/내부 variance 9m²다.",
    "motion gate는 Δ <= k*D+c이며, retained는 이 gate가 유지한 직전 loop에만 anchor를 갱신한다. actual은 역사적으로 삽입된 모든 loop에서 갱신한다.",
    "",
    "| gate | 오류 제외 /6 | 정상 손실 /10 | 구현 범위 |",
    "|---|---:|---:|---|",
])
chosen=[]
for p in summary["profiles"]:
    select=p["name"] in ("original","GUI sigma .19","response coarse .47","fine .50",
        "chain >=20","chain >=40","coarse .42 + sigma .19","chain >=20 + sigma .19")
    select=select or (p["motion_constant_m"]==.2 and p["anchor_policy"]=="retained"
        and p["motion_fraction"] in (.02,.05,.10)
        and p["coarse"]==.35 and p["sigma"] in (3.,.19))
    if not select:
        continue
    name=p["name"].replace("+. ","+")
    scope="Karto source" if p["motion_fraction"] is not None else "config"
    report.append(f"| {name} | {p['wrong_rejected']} | {p['correct_lost']} | {scope} |")
    chosen.append(p)
report.append("| fine >=.48 + sigma .19 | 6 | 4 | config |")
report.append("| 직선 >=70% 퇴화 node 제외 | 0 | 0 | Karto source |")
for threshold in (.35,.40,.45,.50,.55,.60):
    rejected=[r for r in rows if any(
        r[k]["sensor_only"]["dominant_line_fraction"]>=threshold
        and r[k]["sensor_only"]["min_range_m"]<=.5
        for k in ("from_geometry","to_geometry"))]
    wrong=sum(not r["correct"] for r in rejected)
    lost=sum(r["correct"] for r in rejected)
    report.append(f"| 직선 >={threshold*100:.0f}% node 제외 | {wrong} | {lost} | Karto source |")
report.extend([
    "",
    f"- 관측값이 나누는 모든 {frontier['evaluated_splits']:,}개 monotone config 경계를 전수 검사했다. 오류 6개를 모두 제외할 때 정상 최대 {frontier['best_correct_retained']}개 유지·최소 {frontier['correct_lost']}개 손실이다.",
    "- coarse/fine/chain을 높이고 variance를 낮추는 조합만으로 정상 10개를 모두 유지하는 분리는 불가능하다. 예를 들어 새 오류 7:148->284가 정상 6:1679->2088보다 coarse/fine/chain 모두 높고 최대 variance도 낮다.",
    "- 직선 70% 제외와 motion gate를 조합해도 결과는 해당 motion gate와 같다. 퇴화로 분류된 node가 없기 때문이다.",
    "- 5%+0.20m motion의 actual-anchor 결과는 오류 6 제외·정상 4 손실, retained-anchor 결과는 오류 6 제외·정상 3 손실이다. rejection 이후 anchor 정책을 숨기지 않는다.",
    "",
    "## 권고와 승인 후 검증 범위",
    "",
    "설정값만 더 조여 최종 수정으로 삼지 말고, motion consistency를 Karto 삽입 전 gate로 시험하는 방식을 권고한다. 1차 후보는 유지한 loop 기준 D에 대한 5%*D+0.20m이며, 이는 10/50/100m 주행 시 0.70/2.70/5.20m 보정을 허용한다. 고정 0.5m 탐색 제한과 달리 누적 drift에 따라 허용 보정이 증가한다.",
    "",
    "- 이 bound는 실장비에서 검증한 보증값이 아니다. 실제 Go2 odometry drift가 5%*D+0.20m 이내라는 측정이 없으므로 정상 실장비 loop를 전부 유지한다고 주장하지 않는다.",
    "- retained-anchor 고정 후보 재판정에서 손실된 정상은 6:24->1052, 6:422->1203, 6:1031->1322다. 모두 앞선 잘못된 그래프를 복구하는 큰 보정이 포함돼 있다. 첫 오류를 실제로 막은 전체 재생에서도 손실될지는 아직 검증하지 않았다.",
    "- sigma .19를 motion과 조합하면 정상 손실이 3->4개로 늘고 이 고정 표의 추가 오류 제외 이득은 없다. 이미 반영한 .19를 최종 수정으로 확정하지 않는다.",
    "- 응답/chain/variance 방식은 공유 slam_mapping.yaml만으로 가능하다. motion consistency는 MapperGraph::TryCloseLoop에서 pScan->SetSensorPose 및 LinkChainToScan 이전 검사와 odometry arc/anchor 저장을 추가하는 source patch가 필요하다.",
    "- scan 퇴화 제외는 candidate 선택 또는 매칭 전 검사에 source patch가 필요하다. 현재 단일 직선 기준은 오류를 분리하지 못하므로 권고하지 않는다.",
    "- 승인 후에는 motion gate를 오프라인 재생기에만 먼저 구현하고 6차 전체 dataset과 7차 raw scan을 각각 재생해 새로 생긴 모든 제약을 GT 판정해야 한다. 그 결과를 보고하기 전 GUI를 재실행하지 않는다.",
    "",
    "## 상태와 증거",
    "",
    "- GUI는 사용자 지시로 sim 420.368s에 중단했다. owned residue=[], 원본 simulator config 유지=true, 기존 process identity 유지=true다.",
    "- 이전 edge 경보의 원인은 watcher가 이전 파일을 먼저 읽고 native writer가 나중에 truncate한 시작 순서였다. 다음 실행 시작 경계에서 이전 native stream을 비우도록 수정했으나 GUI 재검증은 하지 않았다. 원래 잘못된 alert는 별도 보존했다.",
    "- 20:22:32 진행 기록의 일본어 설명을 한국어로 바로잡았다. 이후 진행 설명은 한국어로 기록했다.",
    "- 이번 지시 이후 제품 설정·코드 추가 변경, GUI 실행, commit, push는 하지 않았다. 생성한 것은 오프라인 분석·보고 자료다.",
    "- 전체 행/9원소 covariance/양쪽 scan 진단: all-accepted-edges.json 및 all-accepted-edges-detailed.csv.",
    "- gate별 전체 비교: gate-comparison.json, exact-config-frontier.json.",
    "- 실제 GUI YAML 및 실행 중 값: attempt-7/gui-params-source.yaml, attempt-7/attempt7-running-params.txt, attempt-7/manifest.json.",
    "- 전체 engine 재생: replay-gui019.log, replay-gui019-edges.jsonl, replay-gui019-factors.jsonl, gui-yaml-full-replay-summary.json.",
    "- 이전 잘못된 경보 보존: attempt7-original-alert-stale-edge.json.",
])
(OUT/"REPORT_KO.md").write_text("\n".join(report)+"\n")
flat=[]
for row in rows:
    item={key:value for key,value in row.items()
          if not isinstance(value,(dict,list)) and key!="fine_link_covariance"}
    item["edge"]=edge(row)
    item["coarse_covariance_9"]=json.dumps(row["coarse_covariance"])
    item["fine_link_covariance_9"]=json.dumps(row["fine_link_covariance"])
    for key in ("from_geometry","to_geometry"):
        item[key]=json.dumps(row[key],ensure_ascii=False)
    flat.append(item)
with (OUT/"all-accepted-edges-detailed.csv").open("w") as stream:
    writer=csv.DictWriter(stream,fieldnames=list(flat[0]))
    writer.writeheader();writer.writerows(flat)
print("GATE_REVIEW_REPORT_WRITTEN "+str(OUT/"REPORT_KO.md"),flush=True)
