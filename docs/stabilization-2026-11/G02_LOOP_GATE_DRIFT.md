<!-- 목적: runtime gate main 통합과 13개 재생·attempt9 drift 원인 분리의 근거를 보존한다.
흐름: 승인 PR/CI → 고정 cohort → 실제 Karto 재생 → GT/지도/파라미터 대조.
입출력: 동결 raw/GT와 native source → 한국어 비교·수정 gate 판단과 증거 참조.
주의: offline 진단 결과이며 실제 로봇 동작·안전 또는 simulation 완료를 뜻하지 않는다. -->
# G02 loop gate와 attempt 9 drift 조사

현재 실행 설정의 변경 여부와 13개 dataset 비교를 검토할 때 사용한다.
선행 [runtime 결과](G02_RUNTIME_GATE.md)와
[12개 counterfactual 결과](G02_COUNTERFACTUAL_REPLAY.md)를 잇는 기록이다.

## 수정 gate 판정

- 판정은 **STOP**이다.
- matching을 끄면 pose drift·doubled가 사라지지만 기존 wall/폭 판정 실패가 남는다.
- 가까운 벽의 관찰은 확인했지만 가장 큰 drift 구간은 기존 단일 벽 퇴화 기준에 해당하지 않는다.
- backend matching drift와 raster/geometry/profile 경계의 잔차를 하나의 확정 원인으로 묶을 수 없다.
- 단일 원인을 A/B로 확정해야 한다는 승인 조건을 충족하지 않는다.
- 제품 수정·추가 fix branch·simulation 설정 적용은 하지 않았다.
- 새 GUI·acceptance·전체 colcon은 조건 미충족으로 실행하지 않았다.
- 이 branch의 변경은 offline 진단·계약 검사·문서다.
- 실물 설정·공유 SLAM YAML·기존 runtime patch·`/opt/ros`는 변경 대상이 아니다.
- simulation 완료와 실제 로봇 동작 검증은 달성했다고 보고하지 않는다.

## 승인된 main 통합

- PR은 [#3](https://github.com/SeoJCalive/go2-local-navigation/pull/3)이다.
- PR head는 `d5156af6c9f276ad3d6b034e8fd0206b7453f2ea`다.
- PR CI [45678858970](https://github.com/SeoJCalive/go2-local-navigation/actions/runs/45678858970)은 success다.
- 앞 PR과 같은 merge commit 방식으로 main에 병합했다.
- merge SHA는 `4f436354e6ae0c8ea101e60cf09abe8ec447316f`다.
- main push CI [45680112421](https://github.com/SeoJCalive/go2-local-navigation/actions/runs/45680112421)은 success다.
- bounded `gh run watch --exit-status` 종료 0과 API의 completed/success를 확인했다.
- AGX main을 해당 SHA로 fast-forward하고 clean 상태를 확인했다.
- 그 main에서 `feat/loop-gate-drift`를 만들었다.
- 통합 영수증은 `.omo/evidence/loop-gate-drift/integration.json`이다.

## 입력과 판정 계약

- 기존 `counterfactual-datasets.json`의 12개 cohort·pin은 그대로 유지한다.
- 별도 `fixtures/drift-datasets.json`이 attempt 9를 13번째로 확장한다.
- attempt 9 입력은 `attempt9-final.data`의 2,092개 scan과 동결 qpos CSV다.
- 원본 `.data` SHA256은 `8b6d0d6e8779fd35207463dfcc5e8a21d85e58c236264caa43a3cb4d8fe2d854`다.
- qpos CSV SHA256은 `ccfb4e6f735b7dc8e31f7c580225b8e3341cef967d5006e01e63984528440521`다.
- route SHA256은 `8b9cbeb85a44c795137447e8350b46aadba1fd21288497dbb1ec12a88207246e`다.
- result·mapping trace·관측 scan도 확장 manifest에 SHA256으로 고정한다.
- 고정 startup world→map은 `[yaw, tx, ty] = [0.000008348973992542905, -0.00020476422346491022, 0.0000050100740033366155]`다.
- 최종 SLAM pose에 transform을 맞추지 않는다.
- attempt 6·9는 독립 qpos GT를 사용한다.
- 나머지 기존 11개는 선행 실험과 같은 serialized simulation odometry를 평가 기준으로 사용한다.
- native source는 slam_toolbox 2.6.10 `8293d21fe5c816d0405e0cc1f3eb46c70659c267`과 기존 격리 replay patch다.
- GT는 native 종료 뒤 평가에만 읽으며 matcher·solver에 주입하지 않는다.

`D`는 연결하려는 두 node 사이 원본 XY odometry path length다.
native가 누적한 query arc에서 anchor가 수락될 때 기록한 arc를 뺀다.
filter로 건너뛴 원본 scan 구간도 arc에 포함한다.

- 필수 정책은 none, odometry XY separation ≤1 m, `correction ≤ kD+c`의 여섯 조합이다.
- `k∈{.03,.05,.08}`, `c∈{.2,.5}`이고 경계는 inclusive다.
- 이전 `(a)` 구현의 D는 마지막 **실제 유지 loop 이후** arc다.
- 혼동을 피하려고 이전 정책을 `retained-0.05-0.2`로 따로 재생한다.
- gate를 거부하면 그 뒤 matching·edge·최적화·지도도 다시 계산한다.
- wrong loop는 실제 생성 edge의 상대 base 측정이 GT 대비 XY >.10 m 또는 yaw >5도인 경우다.
- large wrong은 그중 XY >1 m이며 small wrong은 나머지다.
- 정상 후보 거부는 fine response ≥.45로 실제 gate에 도달한 거부 사건의 수다.
- 같은 source node pair의 반복 제안도 각각 센다.
- sensor extrinsic을 제거한 상대 base 측정으로 후보의 GT 적합성을 판정한다.
- none의 정상 pair 부재는 실제 정상 후보 거부 횟수와 별도 집계한다.
- movement filter가 바꾸는 native ID를 원본 source ID로 복원해 pair를 비교한다.

지도 평가 구현 `geometry.py`와 기존 pass threshold는 유지한다.
occupied-cell wall offset p95 ≤.05 m·max ≤.15 m, thickness p95 ≤.15 m,
normal offset p95 ≤.05 m·max ≤.15 m, normal thickness p95 ≤.15 m,
doubled 0·ghost 0을 함께 검사한다.
strict 비교에서 `.15000000000000013 > .15`도 원래 판정대로 실패다.
pose max·p95는 별도로 보고한다.

2단계 gate는 추가하지 않았다. `sep-horizontal/none`은 실제 loop 0·gate callback
0회인데 wall p95 .343819 m·normal p95 .193750 m·doubled 5·ghost 25다.
gate 수용점에 도달하지 않는 이 실패를 post-fine loop gate만으로 바꿀 수 없다.
높은 response를 추가하는 정책만으로 all-13 지도 기준을 해결할 근거가 없다.

## 13개 전체 집계

13개×9정책의 117개 case다. P95 열은 dataset별 p95의 최댓값이고
pose max도 최댓값이다. Wrong·정상 거부·doubled·ghost는 합계다.
모든 정책이 기존 지도 기준 FAIL이다.

| 정책 | >1 m wrong | small wrong | 정상 후보 거부 | wall p95 m | normal p95 m | doubled | ghost | pose max m | pose p95 m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 11 | 5 | 0 | 1.584338 | .193750 | 42 | 494 | 5.027775 | 4.404311 |
| fixed 1 m | 0 | 0 | 273 | .537505 | .193750 | 50 | 97 | .724970 | .663907 |
| pair .03D+.2 | 2 | 2 | 0 | 1.445983 | .193750 | 52 | 212 | 2.770146 | 2.638955 |
| pair .03D+.5 | 2 | 2 | 0 | 1.445983 | .193750 | 52 | 212 | 2.770146 | 2.638955 |
| pair .05D+.2 | 2 | 2 | 0 | 1.445983 | .193750 | 52 | 212 | 2.770146 | 2.638955 |
| pair .05D+.5 | 2 | 2 | 0 | 1.445983 | .193750 | 52 | 212 | 2.770146 | 2.638955 |
| pair .08D+.2 | 3 | 2 | 0 | 1.445983 | .193750 | 47 | 237 | 2.770146 | 2.638955 |
| pair .08D+.5 | 3 | 2 | 0 | 1.445983 | .193750 | 47 | 237 | 2.770146 | 2.638955 |
| retained .05D+.2 | 0 | 2 | 0 | .537505 | .193750 | 30 | 97 | .724970 | .663907 |

잘못된 loop가 줄어도 doubled count가 단조 감소하지 않는다.
크게 틀어진 지도가 GT profile의 검색 band 밖으로 나가면 count가 줄 수 있으므로,
wrong·wall offset·normal·ghost·pose를 함께 판정한다.

요청한 정책 중 wrong 0을 먼저 만족하는 후보는 고정 1 m뿐이다.
정상 fine 후보 거부는 273회이며 none의 정상 source pair 부재는 25개다.
비례 여섯 정책은 정상 후보 거부 0회여도 changed graph에서 정상 pair 17개가
없고 wrong이 4–5개 남는다. 후보 거부와 최종 edge 보존을 바꿔 쓰지 않는다.

## 원래 12개 cohort의 별도 집계

아래는 attempt 9를 제외한 12×9=108개 case다. Wall/normal/pose p95는
dataset별 지표 중 최대이고 pose max도 dataset별 최대다. Doubled·ghost와
wrong·정상 거부는 사건/성분 수의 합이다. 여러 dataset의 pose를 섞은 p95가 아니다.
모든 행이 기존 지도 기준 FAIL이다.

| 정책 | >1 m wrong | small wrong | 정상 후보 거부 | wall p95 m | normal p95 m | doubled | ghost | pose max m | pose p95 m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 8 | 4 | 0 | 1.584338 | .193750 | 33 | 425 | 5.027775 | 4.404311 |
| fixed 1 m | 0 | 0 | 197 | .537505 | .193750 | 42 | 97 | .724970 | .663907 |
| pair .03D+.2 | 1 | 1 | 0 | .693813 | .193750 | 42 | 141 | 1.024742 | .663907 |
| pair .03D+.5 | 1 | 1 | 0 | .693813 | .193750 | 42 | 141 | 1.024742 | .663907 |
| pair .05D+.2 | 1 | 1 | 0 | .693813 | .193750 | 42 | 141 | 1.024742 | .663907 |
| pair .05D+.5 | 1 | 1 | 0 | .693813 | .193750 | 42 | 141 | 1.024742 | .663907 |
| pair .08D+.2 | 2 | 1 | 0 | .693813 | .193750 | 37 | 166 | 1.024742 | .663907 |
| pair .08D+.5 | 2 | 1 | 0 | .693813 | .193750 | 37 | 166 | 1.024742 | .663907 |
| retained .05D+.2 | 0 | 1 | 0 | .537505 | .193750 | 30 | 97 | .724970 | .663907 |

## Attempt 9의 별도 gate 비교

| 정책 | >1 m wrong | small wrong | 정상 후보 거부 | wall p95 m | normal p95 m | doubled | ghost | pose max m | pose p95 m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 3 | 1 | 0 | 1.024715 | .178750 | 9 | 69 | 3.529558 | 3.271086 |
| fixed 1 m | 0 | 0 | 76 | .079760 | .073750 | 8 | 0 | .394612 | .159595 |
| pair .03D+.2 | 1 | 1 | 0 | 1.445983 | .168750 | 10 | 71 | 2.770146 | 2.638955 |
| pair .03D+.5 | 1 | 1 | 0 | 1.445983 | .168750 | 10 | 71 | 2.770146 | 2.638955 |
| pair .05D+.2 | 1 | 1 | 0 | 1.445983 | .168750 | 10 | 71 | 2.770146 | 2.638955 |
| pair .05D+.5 | 1 | 1 | 0 | 1.445983 | .168750 | 10 | 71 | 2.770146 | 2.638955 |
| pair .08D+.2 | 1 | 1 | 0 | 1.445983 | .168750 | 10 | 71 | 2.770146 | 2.638955 |
| pair .08D+.5 | 1 | 1 | 0 | 1.445983 | .168750 | 10 | 71 | 2.770146 | 2.638955 |
| retained .05D+.2 | 0 | 1 | 0 | .053211 | .056250 | 0 | 0 | .187470 | .097020 |

기존 retained 정책은 doubled/ghost 0이어도 small wrong 1개와 wall/normal
p95 실패가 남아 후보로 채택하지 않는다. Attempt 9의 none 정상 pair 부재는
고정 1 m에서 8개, 비례/retained에서 각 6개다.

## 원본 attempt 9의 구간별 관찰

고정 startup frame에서 같은 native 입력 시각에 qpos를 보간했다.
최초 실제 유지 loop는 781.876 s다.
그 이전 최대 drift를 수락한 false loop의 결과로 설명할 수 없다.

| 비교 | max m | p95 m |
| --- | ---: | ---: |
| original odometry 대 독립 qpos | .000918179476 | .000758982936 |
| SLAM online 대 독립 qpos | .394611847569 | .159595246786 |

| route 구간 | sim s | SLAM 시작→끝 m | 증가 m | odom max m |
| --- | --- | --- | ---: | ---: |
| outer north | 737.394–753.896 | .227701→.381418 | .153717 | .000352 |
| outer north | 695.598–718.998 | .120537→.179354 | .058817 | .000398 |
| outer east | 393.106–404.206 | .116879→.172013 | .055134 | .000583 |
| outer west | 594.298–610.700 | .039794→.093931 | .054137 | .000656 |

- 746.596 s, GT XY `(5.86249, 5.52778)`에서 SLAM 오차 .302523 m·odom .000218 m다.
- 최대 오차 시각은 756.192 s, GT XY `(7.52111, 5.49687)`다.
- 그 시각 SLAM 오차는 .394612 m·odom .000249 m다.
- 첫 실제 loop 781.876 s 직후 SLAM 오차는 .037678 m로 내려간다.
- 상위 15개 증가 구간과 모든 연속 route label 구간은 `drift-segments.json`에 있다.

긴 벽을 따라 증가한다는 관찰과 단일 벽 퇴화의 입증은 구분한다.
기존 sensor-only 판정은 valid endpoint ≥70%가 한 직선의 .03 m 안에 있고
min range ≤.50 m인 경우다. 가장 크게 증가한 25개 scan의 직선 지지율은
median .403599·max .475728, min range .336979 m이며 해당 퇴화 scan은 0개다.
다음 증가 구간 37개도 max .574603·퇴화 0개다.
가까운 벽은 관찰되지만 이 기준으로 featureless 단일 벽을 원인으로 확정하지 않는다.

`Mapper::Process`는 마지막 odometry→corrected 변환을 다음 입력에 전파하고,
movement filter 뒤 sequential MatchScan 결과로 sensor pose·local graph를
만든다. 따라서 입력 odometry가 정확해도 scan-match 보정 오차가 유지될 수 있다.
이 코드 경로만의 추론은 아래 실제 matching-off 대조와 함께 해석한다.

## 파라미터 대조 범위

제품 YAML은 수정하지 않고 replay NodeOptions에만 ROS parameter override를 전달한다.
모든 sensitivity는 고정 1 m gate를 유지한다.
movement distance/heading은 함께 바꾸고 finer resolution은 유효한 smear와 함께
바꾼다. Resolution 효과는 `.01/.03`과 `.005/.03`의 차이로 분리한다.

| 항목 | 현재 값 | 진단 값 |
| --- | --- | --- |
| use scan matching | true | false |
| do loop closing | true | false |
| minimum travel distance / heading | .1 m / .1 rad | .05/.05, .2/.2 |
| correlation dimension | .5 m | .3, .8 |
| correlation resolution | .01 m | .005, smear .03 동시 지정 |
| correlation smear deviation | .1 m | .03 |
| coarse search angle offset | .349 rad | .0698 |
| link fine response | .1 | .3 |
| link maximum distance | 1.5 m | .8 |
| scan buffer size | 10 | 20 |
| buffer maximum scan distance | 5 m | 3 |
| response expansion | true | false |
| distance variance penalty | .5 | .1 |
| angle variance penalty | 1 | .2 |
| scan barycenter | true | false |

아카이브에는 원래 저장된 scan만 있다. minimum travel을 낮춰도 원래 취득 때
버린 ROS scan을 복원하지 못하므로 더 조밀한 live 취득의 검증으로 해석하지 않는다.
높인 movement filter의 pose 지표는 실제 수락된 시각들에 한정한다.

최초 `.005/.1` 단독 대조는 native의 `Smear deviation too small: Must be
between 0.0025 and 0.05`로 SIGABRT했다. 실패한 `search-fine/engine.log`와
첫 sensitivity batch의 종료 1을 보존했다. 유효한 `.005/.03`을 새 이름
`search-fine-smear03`으로 실행하고 완료된 11개 case는 재생하지 않았다.

## Attempt 9 민감도 17개 결과

모든 행의 large/small wrong은 0/0이며 **모든 지도 기준은 FAIL**이다.
Travel coarse만 1,256/2,092개를 수락했고 나머지는 2,092개를 수락했다.
표의 반올림 수치를 판정에 재입력하지 않고 원래 float의 strict 결과를 유지했다.

| 대조 | 정상 후보 거부 | wall p95 m | normal p95 m | doubled | ghost | pose max m | pose p95 m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 기준 fixed 1 m | 76 | .079760 | .073750 | 8 | 0 | .394612 | .159595 |
| scan-off | 0 | .055680 | .038750 | 0 | 0 | .000918 | .000759 |
| loop-off | 0 | .667786 | .168750 | 24 | 67 | 1.238699 | 1.009387 |
| travel-fine | 76 | .079760 | .073750 | 8 | 0 | .394612 | .159595 |
| travel-coarse | 113 | .059469 | .056250 | 6 | 0 | .395646 | .163398 |
| search-narrow | 92 | .064411 | .053750 | 7 | 1 | .460009 | .284815 |
| search-wide | 96 | .072388 | .061250 | 8 | 1 | .461372 | .308778 |
| search-fine-smear03 | 84 | .063792 | .053750 | 5 | 0 | .074222 | .058162 |
| smear-fine | 86 | .056570 | .051250 | 7 | 0 | .067605 | .042625 |
| angle-narrow | 90 | .064367 | .061250 | 5 | 0 | .338298 | .149242 |
| link-response | 89 | .083050 | .086250 | 12 | 0 | .416289 | .159595 |
| link-distance | 350 | .050109 | .056250 | 15 | 0 | .626886 | .339962 |
| buffer-20 | 94 | .084590 | .086250 | 7 | 1 | .420738 | .132451 |
| buffer-distance | 76 | .079760 | .073750 | 8 | 0 | .394612 | .159595 |
| response-expansion-off | 76 | .079760 | .073750 | 8 | 0 | .394612 | .159595 |
| distance-penalty | 91 | .106339 | .106250 | 6 | 1 | .535765 | .266105 |
| angle-penalty | 80 | .069651 | .068750 | 7 | 0 | .436678 | .198122 |
| barycenter-off | 132 | .092411 | .081250 | 4 | 2 | .358693 | .234834 |

matching/loop 자체를 끈 진단 controls를 제외하면 pose 우선 최선은 smear .03이다.
Pose max .394612→.067605 m·p95 .159595→.042625 m로 개선하지만 normal p95
.051250 m·doubled 7·wall/폭 기준이 남는다. Finer grid는 doubled 5로 줄어도
pose .074222 m·wall .063792 m로 smear-only보다 나빠 단일한 지도/pose 승자는 없다.
이 파라미터 대조는 attempt 9만의 관찰이며 튜닝 설정의 13개 일반화 검증으로
승격하지 않았다. STOP 규칙에 따라 설정을 반영하거나 GUI로 넘기지 않았다.

## matching과 raster의 독립 대조

matching-off는 graph와 loop도 끄는 진단이며 제품 후보로 반영하지 않았다.
같은 2,092개 원본 pose가 그대로 나오고 loop 0·doubled 0·ghost 0이다.
그런데 wall p95와 폭 판정은 여전히 통과하지 않는다.

- matching-off의 occupied wall thickness p95는 .2 m다.
- matching-off의 normal thickness p95는 .15000000000000013 m다.
- 같은 matching-off raw endpoint의 실제 scene 표면 거리 p95는 ≤.25 m 비교 band에서 .0151462 m다.
- raw endpoint max는 .0564771 m이며 비교 band ≤.25 m의 821,177개 endpoint다.
- raw endpoint와 raster cell-center wall p95 사이 차이가 남아 있다.
- odometry가 거의 정확해도 현재 raster·geometry/profile 경계에서 실패할 수 있음을 직접 확인했다.
- loop-off는 drift를 더 키우므로 정상 loop 전체가 drift의 원인이라는 해석도 지지하지 않는다.

기존 no-loop cohort의 profile 재검사는 sep-horizontal 5개·sep-wallfullclean 3개·
sep-wallboth 1개의 doubled count를 그대로 재현했다.
앞의 두 사례는 시간에 따라 이동한 raw scan placement이고,
sep-wallboth는 두 실제 면과 5 cm raster/strict .10 m 분리 경계가 겹친 사례다.
원래 map metric이나 판정식을 고쳐 이 count를 없애지 않았다.

## Attempt 9의 doubled 8개를 분해한 근거

같은 raw endpoint를 최종 graph와 원본 odometry에 각각 놓고 벽 법선 좌표를
뺀 값을 Δ로 삼았다. 레이 자체의 3D 투영·양면 관찰 차이를 그대로 공유하므로
SLAM 배치 차이를 따로 볼 수 있다. 표의 u는 해당 벽 중심 기준 접선 좌표다.
통계는 각 counted run strip에 들어오는 endpoint 전체를 대상으로 한다.

| 벽 / u 구간 m | Δ median m | Δ p95 m | abs Δ max m | 대표 interval의 0 횡단 | 대표 중심 간격 m |
| --- | ---: | ---: | ---: | --- | ---: |
| north_band_west / −2.875…3.225 | .034974 | .088105 | .103327 | 있음 | .125 |
| south_band_west / .075….375 | .030168 | .042319 | .049188 | 있음 | .125 |
| south_band_west / .425…1.125 | .030062 | .037951 | .049316 | 있음 | .125 |
| south_band_mid / −2.325…−.975 | .023903 | .032460 | .065053 | 있음 | .10 근방 |
| south_band_mid / −.925…−.175 | .018965 | .029098 | .058420 | 있음 | .10 근방 |
| south_band_mid / −.075….625 | .013271 | .025812 | .049831 | 있음 | .10 근방 |
| south_band_east / .475….775 | .000844 | .035627 | .048142 | 있음 | .10 근방 |
| west_north_room_wall / −1.075…1.025 | .063417 | .083192 | .089485 | 없음 | .10 근방 |

- 8개 모두 모델의 실제 양면은 중심에서 ±.06 m다.
- south 6개는 실제 배치 차이가 있으나 median은 .000844–.030168 m다.
- south 6개의 대표 음수 interval `[-.035,.0025]`가 0을 가로질러 양쪽 side에 중복 배정된다.
- north_band_west의 첫 interval `[-.035,.0525]`도 0을 가로지른다.
- 따라서 7/8개가 실제 양면·raster·side 배정 경계와 결합한다.
- south mid 3개·south east 1개·west north 1개는 대표 중심 간격이 strict .10 m 경계 근방이다.
- 이 5개는 부동소수점 arange/mean의 strict 비교 영향을 받으며 원래 FAIL count를 유지했다.
- north_band_west는 법선 배치 오차 max .103327 m로 가장 크다.
- west_north_room_wall은 median 법선 bias .063417 m이며 실제 지도 위치 오차가 뚜렷하다.
- 후자의 원본 두 면이 같은 양수 side로 옮겨지는 현상도 count와 결합한다.
- 8이라는 count만으로 같은 실제 면의 독립적인 false layer 8개가 생겼다고 결론내리지 않는다.
- matching-off 0개라는 A/B는 배치 오차의 기여를 뒷받침하지만 각 count의 raster 기여와 분리된 인과 분율은 확정하지 않는다.

각 배치의 표면 거리 ≤.25 m 비교 band에서 raw endpoint p95는 최종 graph
.058655 m·odometry .015146 m다. 두 배치의 선택 표본 수는 각각 821,177개다.
재생한 PGM의 occupied-cell p95는 .079760 m다.
선행 GUI에서 저장한 PGM의 .075041 m와 동일한 숫자로 취급하지 않는다.
이번 비교는 같은 native 재생 방식끼리의 비교이며 attempt 9의 실제 runtime과
PGM byte 동등성을 새로 주장하지 않는다. 원래 attempt 6 재현 검사의 선언된
similarity band와 엄격한 지도 합격 기준도 서로 다른 검사다.

## 변경 파일과 실행 경계

아래는 이 branch의 tracked 변경 10개다. 각 파일 header에 역할과 경계를 설명했다.
진단 실행은 ROS domain 198·localhost이며 robot command를 publish하지 않는다.

| 파일 | 역할 / caller / 입출력 |
| --- | --- |
| `src/go2_mujoco_lidar/test/wall_hug/counterfactual/engine_replay.cpp` | 기존 native CLI에 pair arc·선택 관측·수치/bool ROS override를 추가한다. Raw Dataset→실제 matcher/graph/map이며 제품 SDK는 설치하지 않는다. |
| `src/go2_mujoco_lidar/test/wall_hug/counterfactual_replay.py` | 기존 orchestration에서 고정 startup truth와 online pose p95를 공유한다. 기존 12개 기본 정책·평가 코드를 유지한다. |
| `src/go2_mujoco_lidar/test/wall_hug/counterfactual_profiles.py` | 기존 profile CLI를 재사용하고 명시한 transform·scan 입력·최종 graph를 지원한다. 기본 no-loop assert를 유지하며 raw 법선 차이를 출력한다. |
| `src/go2_mujoco_lidar/test/wall_hug/drift_replay.py` | 13개 gate·17개 대조 CLI다. Native 결과→실제 거부 GT·cohort별 집계·route/벽 근거이며 완료 checkpoint와 source ID를 관리한다. |
| `src/go2_mujoco_lidar/test/wall_hug/fixtures/drift-datasets.json` | 원본 12개를 참조하고 attempt 9 raw/GT/trace/result/route의 SHA와 고정 transform을 추가한다. |
| `src/go2_mujoco_lidar/test/test_wall_hug_drift.py` | Cohort/grid/extrinsic·native arc/결정성·matching-off·source ID·유효 smear 계약이다. Native 자산 없는 CI는 native 검사만 명시 skip한다. |
| `src/go2_mujoco_lidar/CMakeLists.txt` | 기존 ament/CTest에 `wall_hug_drift`를 등록한다. 기존 검사를 삭제하거나 완화하지 않는다. |
| `src/go2_mujoco_lidar/test/wall_hug/README.md` | 입력·build·재생·중단 후 동일 binary 재개·GT/집계 경계를 안내한다. |
| `docs/stabilization-2026-11/G02_LOOP_GATE_DRIFT.md` | 이 실험의 승인·수치·STOP·검증 한계를 보존한다. |
| `docs/stabilization-2026-11/README.md` | 새 기록의 탐색 링크를 추가한다. |

## 재현과 증거

- 실행 방법은 [하니스 README](../../src/go2_mujoco_lidar/test/wall_hug/README.md)의 drift 절에 있다.
- 이번 원본 evidence root는 `.omo/evidence/loop-gate-drift/`다.
- `runs/<dataset>/<variant>/`에 command·실제 factor·edge·decision·online/final pose·PGM·map metrics·overlay를 보존한다.
- `runs/comparison.json`은 original 12·attempt 9·all 13 집계를 분리한다.
- `runs/sensitivity.json`은 attempt 9의 파라미터별 대조다.
- 정상 거부 후보별 sim 시각·native pair·response·GT 오차는 각 summary에 있다.
- native ID와 원본 source ID 대응은 같은 run의 online JSONL에 있다.
- `original-scan-off-profile.json`은 matching-off raw endpoint/지도 경계 대조다.
- `profiles-regression/`은 기존 5/3/1 no-loop profile 재현이다.
- `build-final.log`는 격리 native 엔진 빌드 종료 0이다.
- `drift-ctest.log`는 정식 CTest wrapper 종료 0이다.
- `counterfactual-regression.log`는 기존 16개 검사 PASS다.
- `source-pair-contract.log`는 filter의 ID 재번호화 계약 검사 PASS다.
- `resolution-smear-contract.log`는 유효한 finer resolution/smear 대조 계약 PASS다.
- 새 고유 검사 6개와 기존 counterfactual 16개, 합계 22개가 PASS다.
- 정식 CTest wrapper는 당시 4개를 실행했고 추가한 2개 계약은 각각 별도 pytest로 실행했다.
- `comparison.log`와 `sensitivity-resumed.log`의 최종 batch는 모두 종료 0이다.
- 성공한 science case는 117+17=134개이며 거부된 파라미터 시도 1건은 별도 보존한다.
- `result-integrity.json`과 `result-integrity.log`는 117/17개 누락 없음·startup frame·source pair 복원·집계 일치를 확인해 종료 0이다.
- `profiles-final-regression.log`는 raw 진단 필드 추가 후 이번 native 재생 3개 cohort의 기본 no-loop profile 5·3·1을 확인해 종료 0이다.
