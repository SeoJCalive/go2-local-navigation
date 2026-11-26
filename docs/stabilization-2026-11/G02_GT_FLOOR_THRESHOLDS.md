<!-- 목적: GT pose floor로 보정한 G02 지도 기준과 simulation pose 우선 설정을 설명한다.
입력: GT 6/7/9차와 attempt9 native 대조. 출력: 명시한 한계와 완주별 판정 계약.
주의: simulation 한정이며 실물 설정·동작·안전의 검증 근거가 아니다. -->
# G02 GT 하한 보정과 pose 우선 설정

2026-11-26 소유자 승인 범위다. 지도 해상도 .05 m에서 기존 wall/normal
p95 .05 m는 정확히 한 셀이며, 독립 qpos로 pose를 고정해도 기존 한계를
충족하지 못했다. matcher 오차와 occupancy·raster 측정 하한을 구분한다.

| 지표 | 독립 GT 관찰 | 승인한 한계 | 여유 |
| --- | --- | --- | --- |
| wall offset p95 | 6차 .056019155 m, 9차 .055566273 m | .085 m | 하한 최댓값 + 반 셀 .025 m 이상 |
| normal profile offset p95 | 부분 7차 .041250 m, 9차 .038750 m | .070 m | 하한 최댓값 + 반 셀 .025 m 이상 |
| occupied wall thickness p95 | 6·7·9차 .20 m | .25 m | 한 셀 .05 m |
| normal profile thickness p95 | 6·7차 .20 m, 9차 .15 m | .25 m | 관찰 최댓값 + 한 셀 .05 m |

이 여유는 통계적 신뢰구간이나 다른 환경의 보장이 아니다.
GT 원본·반복 결정성·직접 Karto raster 근거는 AGX의
`.omo/evidence/gt-pose-floor/floor-summary.json`과 각 `g02-*/repeat-*`에 있다.

- 완주한 경로는 `doubled_wall_components = 0`을 요구한다.
- 부분·중단 경로는 doubled count를 보고하되 gate에 넣지 않는다. 부분 GT 7차의 count는 6이다.
- ghost 0, 두 offset max .15 m, pose max .10 m, .30 m alert, contact와 Nav2 등 다른 기준은 유지한다.
- `geometry.map_criteria(..., complete_route=...)`가 지도 기준을 공유한다. 기본값은 부분 경로이며 CLI 완주는 `--complete-route`로 명시한다.
- GUI runner는 driver `finished`가 참일 때만 완주로 판정한다.

pose 우선 설정은 `344e0c7`의 smear-fine 대조에서 선택한
`correlation_search_space_smear_deviation = .03` 한 값이다.
main의 simulation mapping launch override에만 넣으며 공유·실물
`src/go2_nav2/config/slam_mapping.yaml`의 .1과 1 m runtime gate는 유지한다.
attempt9 대조에서 pose max/p95는 .394612/.159595→.067605/.042625 m,
wall/normal p95는 .056570/.051250 m였지만 doubled 7개는 남았다.
새 임계값으로 doubled를 지우거나 추가 튜닝하지 않는다.

PR 전 offline replay, attempt9 재평가와 main GUI 1회·acceptance 1회·전체 test의
실제 결과는 `.omo/evidence/g02-gt-floor-completion/` 및 로컬 위키의 날짜 있는
실험 record에 보존한다. 과거 문서의 기존 FAIL은 당시 기준의 역사로 유지한다.
