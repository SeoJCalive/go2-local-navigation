<!-- 목적: 실제 두 면을 모델링한 G02 이중 벽 판정의 근거와 반례 검증.
입력: MJCF 실제 두께·저장 GT/SLAM 지도·연속 node pose 이동.
출력: 면 배정·허용치·이전/이후 count. simulation 한정이며 실물 근거가 아니다. -->
# G02 실제 두 면 기준 이중 벽 판정

2026-11-26 승인된 simulation 측정기 변경이다. 기존 run 10의 10개 count는
같은 면의 복제가 아니라 실제 반대 면의 두 occupied interval이었다.
음수 면의 sub-cell bias와 raster가 interval을 중심선 너머로 옮기면 이전
측정기는 그 interval을 양 side에 넣었다. 그 결과 실제 양면을 같은 side의
복제로 세었다. 노드별 원인 근거는 `.omo/evidence/g02-node-causality/`에 있다.

`geometry.evaluate`는 각 box의 MJCF `size`에서 실제 반두께 `minor`를 읽고,
각 occupied run 중심을 `-minor`, `+minor` 중 가까운 실제 면에 한 번 배정한다.
동률은 음수 면이다. 중심선을 가로지른 run을 두 번 배정하지 않는다.
분리된 run이 둘 이상이고 그중 하나라도 가까운 실제 면에서 허용치보다 멀면
설명되지 않는 추가 run이다. 이 profile이 기존과 같이 0.20 m 이상 이어지면
component 하나로 센다. 하나뿐인 이동 run은 offset 지표가 판단한다.

허용치는 [기존 GT 보정](G02_GT_FLOOR_THRESHOLDS.md)의 normal p95 한계
**0.070 m를 그대로 사용**한다. 새 scalar threshold를 만들거나 기존 한계를
늘리지 않았다. 0.05 m raster의 축 방향 반 셀 0.025 m를 빼면 nominal
pose 이동 예산은 `0.070 - 0.025 = 0.045 m`다. 이는 임의로 정한 4–5 cm가
아니라 승인된 측정 예산의 분해다. 회전 raster·GT 보간·ray clearing까지
보장하는 수학적 상한은 아니므로 아래 실제 Karto 합성 이동으로 확인했다.
정상 run의 잔차를 p95로 잘라 버리지 않고 각 run 중심에 같은 허용치를 적용한다.

- complete route의 doubled 한계는 계속 **0**이다.
- partial route는 count만 보고한다. 부분 GT 7차의 이전 6개는 새 판정에서 0이다.
- wall/normal offset, 두 thickness, ghost, pose, contact와 Nav2 threshold는 그대로다.
- 모든 비교에서 doubled count와 벽별 count 외의 geometry 출력은 정확히 같았다.
- 결정론 시험은 실제 두께별 nearest-face 배정, 중심선 교차, 양쪽 이동과 strict
  허용치 경계, 기존 정확/이동/이중/ghost/두꺼운 지도 대조를 포함해 3 passed다.

## 저장 지도 대조

전체 receipt는 `.omo/evidence/g02-doubled-wall-faces/validation.json`이다.
기존 `e8fea28` 측정기를 Git에서 읽어 같은 입력에 적용했다. 새 SLAM 최적화나
GT 재정렬은 하지 않았다.

| 지도 | 이전 doubled | 새 doubled |
| --- | ---: | ---: |
| GT 6 | 0 | 0 |
| GT 7, 부분·비게이트 | 6 | 0 |
| GT 9 | 0 | 0 |
| GT run 10 | 0 | 0 |
| 실제 저장 attempt 6, false-loop collapse | 9 | 19 |
| none g02-6 | 9 | 19 |
| none sep-boundary | 1 | 4 |
| none sep-cause | 1 | 4 |
| none sep-colored | 2 | 3 |
| none sep-horizontal | 5 | 11 |
| none sep-toggle | 2 | 3 |
| none sep-wallclean | 1 | 7 |
| none sep-wallfullclean | 3 | 8 |
| none sep-westspine | 0 | 5 |
| none sep-wallboth | 1 | 0 |
| 실제 저장 run 10 | 10 | 0 |

`sep-wallboth`는 pose max 약 0.070 m·wall p95 약 0.032 m인 낮은 오차의
양면 raster 대조다. 이전 문서도 이를 판정 경계로 구분했다. 큰 pose 오차나
false-loop collapse 음성 대조로 바꾸어 해석하지 않았다. 모든 known-bad의
count는 양수이며, 과거 near=.20 m cutoff 밖의 추가 run도 측정하므로 일부는
이전보다 늘었다. normal offset/폭의 기존 near 선택 범위는 바꾸지 않았다.

2026-11-26 소유자가 known-bad 검증 기준을 정정하고 병합을 승인했다.
`gate=none` 자체는 불량 지도라는 뜻이 아니다. `sep-wallboth`는 wrong loop 0,
pose max .0698 m, wall p95 .032 m이며, [기존 원인 표](G02_COUNTERFACTUAL_REPLAY.md#오류-loop-0이어도-남는-이중벽의-원인)의
465행에서 한 개의 이전 doubled를 실제 두 면·raster·중심선 교차 artifact로
분류했다. 따라서 이 지도의 1→0과 지도 gate 통과는 올바른 양성 대조다.
known-bad 집합은 wrong loop가 있거나 pose 오차가 큰 지도이며, 위 표의 해당
지도는 모두 새 기준에서도 FAIL이다. 앞선 모든 none FAIL 요구에 따른 STOP은
검증 기준의 오류였고 검출기 결함이 아니며, 다른 threshold 변경 없이 정정했다.

## 연속 GT node pose 이동

run 10 독립 qpos GT의 2,123 scan 전체를 같은 Karto
`CreateFromScans(.05, 2, .1)`에 넣었다. 한 면의 hit가 있는 가장 긴 연속 관측
episode에서 중앙 절반(최대 80개)의 **node XY pose 전체**를 선택한 면의
법선 바깥 방향으로 이동했다. 다른 node, yaw, ranges는 그대로다. 양 끝의
미이동 관측을 실제 면 reference로 남겼다. pixel을 직접 옮기지 않았다.

| 벽 | 연속 node ID, 0 기준 | scan 시각 s | 실제 두께 m | 방향 world XY |
| --- | --- | --- | ---: | --- |
| south_band_east, 얇은 양면 | 100–176, 77개 | 300.336–350.336 | .12 | [0, 1] |
| east_south_angled_wall | 299–317, 19개 | 431.330–443.528 | .14 | [.500000194, -.866025292] |
| outer_north, 안쪽 단면 | 1568–1647, 80개 | 1277.816–1330.318 | .10 | [0, -1] |

아래는 **이전→새 전체 count (새 대상 벽 count)**다. 모든 24개 결과를 보고한다.
0.02/0.03/0.04 m는 .045 m 이동 예산 이하여서 비검출을 요구했다.
0.06/0.08 m는 예산 밖이며 두 판정 중 하나를 미리 강제하지 않았다.
0.10/0.15/0.20 m는 세 대상 벽 모두 검출을 요구했고 전부 통과했다.

| 이동 m | 얇은 양면 | 기울어진 벽 | 외곽 단면 |
| ---: | --- | --- | --- |
| .02 | 0→0 (0) | 0→0 (0) | 0→0 (0) |
| .03 | 0→0 (0) | 0→0 (0) | 0→0 (0) |
| .04 | 0→0 (0) | 0→0 (0) | 0→0 (0) |
| .06 | 0→0 (0) | 0→0 (0) | 2→0 (0) |
| .08 | 0→0 (0) | 0→0 (0) | 4→0 (0) |
| .10 | 1→1 (1) | 0→1 (1) | 3→2 (1) |
| .15 | 1→1 (1) | 13→2 (2) | 3→2 (1) |
| .20 | 0→1 (1) | 14→2 (2) | 4→3 (1) |

outer selection의 정확한 node·시각, 모든 입력/기존 지표와 벽별 count는
`validation.json`의 `selections`, `shifts`에 있다. 이 검사는 선택한 외향 이동의
관찰이며 모든 방향·grid phase·벽·관측 밀도의 일반 검출 보장은 아니다.

## run 10 저장 지도 재판정

| 지표 | 결과 m 또는 개수 | 기존 한계 | 판정 |
| --- | ---: | ---: | --- |
| wall offset median / p95 / max | 0 / .065220246 / .105132379 | p95 .085 / max .15 | PASS |
| normal offset median / p95 / max | .01875 / .06625 / .08125 | p95 .070 / max .15 | PASS |
| occupied thickness p95 / max | .20 / .30 | p95 .25 | PASS |
| normal thickness p95 / max | .15 / .2125 | p95 .25 | PASS |
| ghost / doubled | 0 / 0 | 0 / 0 | PASS |

- occupied cell: 4,717; resolution: .05 m.
- world→map `[yaw,tx,ty]`: `[-.0000172068056867123,-.000288986129974244,.0000012378283443628983]`.
- 지도 gate는 8/8 PASS다. count 이외 지표는 원래 저장 지도와 정확히 같다.
- 이전 GUI의 pose max .072943808 m, 경로 완주, contact 정책 통과 관찰은 기존
  실행 기록이며 새 GUI 관찰로 대신하지 않는다.
- PR/CI/main GUI/acceptance/전체 colcon과 최종 감사는 이 오프라인 사전 검증 뒤
  별도로 실행하며 날짜 있는 최종 record가 실제 결과를 보존한다.
