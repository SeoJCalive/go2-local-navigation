# G02 counterfactual loop gate 재생

2026-11-25 승인된 오프라인 비교의 입력·엔진·관측·실물 적용 한계를 조회할 때 사용한다.
근거 등급은 observed·simulation 한정이다. 실행 브랜치는 feat/loop-gate-replay이고 main에 병합하지 않는다.

## 입력과 재생 가능성

[고정 manifest](../../src/go2_mujoco_lidar/test/wall_hug/fixtures/counterfactual-datasets.json)에
10개 .data, 독립 physics truth, world와 원본 parameter 파일의 SHA256을 기록했다.
9월 .data는 posegraph뿐 아니라 raw range·laser 정의·원래 odometry를 보존했다.
각 재생은 저장된 corrected pose를 버리고 odometry로 초기화한 빈 graph에서 시작한다.

- G02 6차: 2,099 scan, 독립 physics qpos GT가 있다.
- 9월: cause·toggle·horizontal·wallclean·wallfullclean·boundary·colored·westspine·wallboth 9개다.
- 9월 GT는 직렬화된 simulation odometry다. 별도 독립 qpos GT로 오해하지 않는다.
- sep-wallclean은 당시 stale publisher 가능성 때문에 비정식인 조건을 유지한다.
- G02 7차: raw scan·candidate·qpos는 있지만 원본 odometry가 없다. qpos를 odometry 입력으로 대체한 결과는 실제 odometry 재생으로 셀 수 없어 제외했다. fine covariance 부재도 그대로 보존한다.
- 9월 wallclose 120906·121514: 로그·이미지는 남았지만 .data 또는 동등한 전체 ranges+odometry stream은 없다.
- .data에는 당시 movement filter가 처리한 scan이 있다. 당시 버린 모든 ROS scan이 보존됐다는 뜻은 아니다. 이번 baseline 10개는 movement_filtered=0이다.

## 엔진과 인과 경계

기준 엔진은 slam_toolbox 2.6.10 / 8293d21fe5c816d0405e0cc1f3eb46c70659c267,
AGX Ceres 2.0.0, TBB 2021.5.0이다. 기존 native replay_dataset.cpp를 출발점으로
[정식 CLI](../../src/go2_mujoco_lidar/test/wall_hug/counterfactual_replay.py)와
[repo-only patch](../../src/go2_mujoco_lidar/test/wall_hug/counterfactual/karto-gate.patch)를 만들었다.
빌드·실행 방법은 [runbook](../../src/go2_mujoco_lidar/test/wall_hug/README.md#counterfactual-엔진-재생)에 있다.

- prefix: .omo/evidence/loop-gate-replay/engine. 제품 install과 /opt/ros에는 설치하지 않았다.
- sigma 3.0, 응답 .35/.45, chain 10과 원본 mapping YAML을 유지한다. 실행 시 다른 sigma는 오류다.
- Karto·Ceres worker를 각각 1개로 고정했다. 과거 query prior·GT·수락 결정·preload hook을 주입하지 않았다.
- fine 수락 뒤 원본 scan을 바꾸기 전에 정확히 같은 barycenter 기준의 linked anchor를 구한다. 거부하면 pose 변경·삽입·최적화를 건너뛰고 다음 candidate를 찾는다.
- 거리 gate는 linked anchor와 query의 원본 odometry XY 차이가 X 이하인지 본다. 보정 gate는 fine best sensor pose와 현재 query sensor pose의 XY 차이가 kD+c 이하인지 본다. 조합은 두 조건 모두를 요구한다.
- D는 첫 scan 또는 마지막 실제 새 loop 삽입부터 원본 odometry XY arc다. 거부와 기존 edge callback은 anchor를 reset하지 않는다.
- map은 동일 Karto occupancy 생성기의 0.05 m·min_pass_through=2·occupancy=.1 결과이며 기존 geometry.evaluate로 판정한다. 최종 graph에 맞춘 GT 재정렬은 하지 않는다.
- 각 실행의 graph·PGM/YAML·factor·gate 결정·online/final pose·overlay·metrics·summary는 .omo/evidence/loop-gate-replay/runs-isolated에 보존한다.

첫 build에서는 ROS LD_LIBRARY_PATH가 공유 SDK를 우선 선택했다. 그 산출물은 runs/에
별도로 보존하고 비교에서 제외했다. 전용 SDK 우선 RPATH와 강한 patch-version symbol을
추가한 뒤 ldd로 실제 전용 library를 확인했다. 실제 native 회귀는 거리 gate가
140→275의 삽입과 이후 pose를 바꾸는 것까지 검사하여 무효한 재판정과 구별한다.

## gate=none 재현

6차 최초 오류는 140→275 / 201.302 s이며 immutable 상대 XY 오차는
3.553811215843413 m다. 실제 15개 중 오류 5개·정상 10개를 얻었다.

| 지표 | 원래 기록 | 새 graph 재생 |
| --- | ---: | ---: |
| 벽 offset p95 m | 1.062681 | 1.061744 |
| doubled / ghost | 9 / 75 | 9 / 75 |
| online pose 최대 XY m | 5.027770 | 5.027775 |

선언한 재현 범위는 첫 edge·시각 일치, p95 ±.22 m·doubled ±4·ghost ±20·
최대 pose 오차 ±.55 m다. 실제 차이는 위 표와 같이 훨씬 작다.
후반의 한 loop pair는 원래 1237→1926 대신 1238→1925다. 전체 과거 edge list를
강제로 복원하는 prior hook을 사용하지 않았고, 첫 오류·오류 수·지도 지표의 재현으로 판정했다.
9월 cause도 정상 69→186과 오류 177→300을 재현했다. 과거 toggle dataset의 none은
과거 2 m 토글을 다시 적용하지 않은 graph이므로 그때 저장된 최종 map과 같다고 주장하지 않는다.

| dataset | scan | 오류 / 정상 삽입 | p95 m | doubled / ghost | online pose max m |
| --- | ---: | ---: | ---: | ---: | ---: |
| g02-6 | 2099 | 5 / 10 | 1.061744 | 9 / 75 | 5.027775 |
| sep-cause | 302 | 1 / 1 | 0.983311 | 1 / 25 | 1.890487 |
| sep-toggle | 204 | 2 / 0 | 1.243856 | 2 / 45 | 3.746897 |
| sep-horizontal | 280 | 0 / 0 | 0.343819 | 5 / 25 | 0.467467 |
| sep-wallclean | 309 | 0 / 0 | 0.537505 | 1 / 29 | 0.672665 |
| sep-wallfullclean | 306 | 0 / 0 | 0.297898 | 3 / 25 | 0.474472 |
| sep-boundary | 306 | 1 / 1 | 1.025460 | 1 / 34 | 2.240657 |
| sep-colored | 269 | 1 / 1 | 0.817939 | 2 / 24 | 2.823264 |
| sep-westspine | 333 | 0 / 0 | 0.497290 | 0 / 18 | 0.724970 |
| sep-wallboth | 118 | 0 / 0 | 0.032033 | 1 / 0 | 0.069841 |

## 지표 해석

- 오류/정상은 immutable LinkInfo 상대 측정과 GT 상대 pose의 XY≤.10 m·yaw≤5°로 분류한다. linked node의 실제 거리 자체가 정답 라벨은 아니다.
- 정상 손실은 none의 정상 동일 node pair가 새 graph에 없는 개수다. 다른 정상 anchor로 바뀐 경우도 포함되므로 새 정상 삽입 수도 함께 보고한다. 이를 모두 직접적인 gate 거부라고 부르지 않는다.
- pose max는 각 scan 처리·최적화 직후의 online corrected base pose와 같은 시각 GT의 XY 오차다. 최종 graph 오차도 개별 summary에 보존한다.
- 아래 합계는 같은 10개 dataset을 합친 값이다. 지도 p95·pose는 dataset별 최댓값, doubled·ghost는 합계다. 개별 수치는 상세 표를 따른다.
- loop 0인 9월 baseline에서도 큰 지도 잔차가 남는다. loop gate만으로 전체 지도 수용 기준의 합격을 입증한 것은 아니다.

## 전체 15 gate × 10 dataset 결과와 선택

기준 10회·독립 gate sweep 130회·조합 10회를 모두 끝냈다. sigma는 전 실행 3.0이다.
선택 순서는 전체 엄격한 GT 오류 수, G02 online pose max, G02 p95, 정상 삽입 수이며,
완전 동률이면 X·c·k가 큰 조건을 택했다. 거리 최선은 1.0 m, 보정 최선은 5%D+.3 m다.
3%·5%와 각 c=.1/.2/.3은 요청 집계 지표에서 모두 같아 보정 gate의 동률을
더 느슨한 5%D+.3 m로 풀었다. 이를 결합해 10개 전체를 다시 재생했다.

**시뮬레이션에서 엄격한 오류 0을 우선하면 거리 1.0 m 단독을 권고한다.**
전체 오류는 none 10개에서 0개로 줄었고 새 graph 정상 총수는 13개에서 15개다.
G02 p95는 1.061744→0.057100 m, ghost 75→0, online pose max 5.027775→0.269157 m다.
거리+보정 조합은 10개 각각의 요청 지표와 loop 목록이 거리 1.0 m 단독과 같았다.
따라서 이번 자료에서 조합을 추가할 이득은 관찰하지 못했다.
이 권고는 실제 odometry drift가 측정되지 않은 실물 설정 확정을 뜻하지 않는다.

오류 괄호는 그중 immutable 상대 XY 오차 >1 m인 수다. 작은 기준 위반과
큰 위치 혼동을 구별하기 위한 보조 지표이며 기존 정상 기준을 바꾸지 않는다.
정상 pair 부재는 대체 anchor 선택도 포함한다. 예를 들어 거리 1 m의 12개 부재를
정상 closure 기회 12개 상실로 해석할 수 없고, 새 graph 정상 총수 15개와 함께 본다.
전체 p95·pose max는 dataset별 최댓값, doubled·ghost는 합이다.

| gate | 오류 (XY>1 m) | 기준 정상 pair 부재 | 새 graph 정상 총 | p95 max m | doubled 합 | ghost 합 | online pose max m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 10 (6) | 0 | 13 | 1.243856 | 25 | 300 | 5.027775 |
| 거리 1 m | 0 (0) | 12 | 15 | 0.537505 | 31 | 97 | 0.724970 |
| 거리 1.5 m | 1 (1) | 12 | 15 | 0.537505 | 34 | 115 | 0.936890 |
| 거리 2 m | 1 (1) | 12 | 16 | 1.630722 | 33 | 161 | 2.759351 |
| 거리 3 m | 8 (6) | 9 | 11 | 4.004892 | 20 | 297 | 11.792027 |
| 3%D+0.1 | 1 (0) | 9 | 14 | 0.537505 | 21 | 97 | 0.724970 |
| 3%D+0.2 | 1 (0) | 9 | 14 | 0.537505 | 21 | 97 | 0.724970 |
| 3%D+0.3 | 1 (0) | 9 | 14 | 0.537505 | 21 | 97 | 0.724970 |
| 5%D+0.1 | 1 (0) | 9 | 14 | 0.537505 | 21 | 97 | 0.724970 |
| 5%D+0.2 | 1 (0) | 9 | 14 | 0.537505 | 21 | 97 | 0.724970 |
| 5%D+0.3 | 1 (0) | 9 | 14 | 0.537505 | 21 | 97 | 0.724970 |
| 10%D+0.1 | 3 (2) | 8 | 13 | 1.137810 | 21 | 181 | 5.027775 |
| 10%D+0.2 | 3 (2) | 8 | 13 | 1.137810 | 21 | 181 | 5.027775 |
| 10%D+0.3 | 3 (2) | 8 | 13 | 1.137810 | 21 | 181 | 5.027775 |
| 거리 1 ∧ 5%D+0.3 | 0 (0) | 12 | 15 | 0.537505 | 31 | 97 | 0.724970 |

## dataset별 전체 150개 실행

아래 값은 각 재최적화 graph와 최종 map의 실제 측정이다. 동일한 숫자도
해당 gate의 별도 엔진 재생 결과다. 모든 graph·map·decision·원본 arc는 AGX의
.omo/evidence/loop-gate-replay/runs-isolated에서 dataset/gate별로 조회한다.
기계가 읽는 전체 집계는 comparison.json, 선택 기준과 두 최선은 selection.json이다.
최초 SDK 선택이 잘못된 runs/의 산출물은 이 150개에 포함하지 않았다.

| dataset | gate | 오류 삽입 | 기준 정상 pair 부재 | 새 graph 정상 총 | p95 m | doubled | ghost | online pose max m |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| g02-6 | none | 5 | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| g02-6 | 거리 1 m | 0 | 10 | 11 | 0.057100 | 6 | 0 | 0.269157 |
| g02-6 | 거리 1.5 m | 0 | 10 | 11 | 0.059109 | 9 | 0 | 0.138086 |
| g02-6 | 거리 2 m | 1 | 10 | 12 | 1.630722 | 8 | 64 | 2.759351 |
| g02-6 | 거리 3 m | 4 | 9 | 7 | 4.004892 | 5 | 86 | 11.792027 |
| g02-6 | 3%D+0.1 | 1 | 9 | 10 | 0.065271 | 1 | 0 | 0.113926 |
| g02-6 | 3%D+0.2 | 1 | 9 | 10 | 0.065271 | 1 | 0 | 0.113926 |
| g02-6 | 3%D+0.3 | 1 | 9 | 10 | 0.065271 | 1 | 0 | 0.113926 |
| g02-6 | 5%D+0.1 | 1 | 9 | 10 | 0.065271 | 1 | 0 | 0.113926 |
| g02-6 | 5%D+0.2 | 1 | 9 | 10 | 0.065271 | 1 | 0 | 0.113926 |
| g02-6 | 5%D+0.3 | 1 | 9 | 10 | 0.065271 | 1 | 0 | 0.113926 |
| g02-6 | 10%D+0.1 | 2 | 8 | 9 | 1.137810 | 6 | 59 | 5.027775 |
| g02-6 | 10%D+0.2 | 2 | 8 | 9 | 1.137810 | 6 | 59 | 5.027775 |
| g02-6 | 10%D+0.3 | 2 | 8 | 9 | 1.137810 | 6 | 59 | 5.027775 |
| g02-6 | 거리 1 ∧ 5%D+0.3 | 0 | 10 | 11 | 0.057100 | 6 | 0 | 0.269157 |
| sep-cause | none | 1 | 0 | 1 | 0.983311 | 1 | 25 | 1.890487 |
| sep-cause | 거리 1 m | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 거리 1.5 m | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 거리 2 m | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 거리 3 m | 1 | 0 | 1 | 0.983311 | 1 | 25 | 1.890487 |
| sep-cause | 3%D+0.1 | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 3%D+0.2 | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 3%D+0.3 | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 5%D+0.1 | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 5%D+0.2 | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 5%D+0.3 | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 10%D+0.1 | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 10%D+0.2 | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 10%D+0.3 | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | 거리 1 ∧ 5%D+0.3 | 0 | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-toggle | none | 2 | 0 | 0 | 1.243856 | 2 | 45 | 3.746897 |
| sep-toggle | 거리 1 m | 0 | 0 | 1 | 0.060677 | 3 | 0 | 0.115290 |
| sep-toggle | 거리 1.5 m | 0 | 0 | 1 | 0.071007 | 4 | 0 | 0.115290 |
| sep-toggle | 거리 2 m | 0 | 0 | 1 | 0.047769 | 6 | 0 | 0.115290 |
| sep-toggle | 거리 3 m | 1 | 0 | 1 | 0.422403 | 0 | 25 | 0.955205 |
| sep-toggle | 3%D+0.1 | 0 | 0 | 1 | 0.036679 | 5 | 0 | 0.114857 |
| sep-toggle | 3%D+0.2 | 0 | 0 | 1 | 0.036679 | 5 | 0 | 0.114857 |
| sep-toggle | 3%D+0.3 | 0 | 0 | 1 | 0.036679 | 5 | 0 | 0.114857 |
| sep-toggle | 5%D+0.1 | 0 | 0 | 1 | 0.036679 | 5 | 0 | 0.114857 |
| sep-toggle | 5%D+0.2 | 0 | 0 | 1 | 0.036679 | 5 | 0 | 0.114857 |
| sep-toggle | 5%D+0.3 | 0 | 0 | 1 | 0.036679 | 5 | 0 | 0.114857 |
| sep-toggle | 10%D+0.1 | 1 | 0 | 1 | 0.422403 | 0 | 25 | 0.955205 |
| sep-toggle | 10%D+0.2 | 1 | 0 | 1 | 0.422403 | 0 | 25 | 0.955205 |
| sep-toggle | 10%D+0.3 | 1 | 0 | 1 | 0.422403 | 0 | 25 | 0.955205 |
| sep-toggle | 거리 1 ∧ 5%D+0.3 | 0 | 0 | 1 | 0.060677 | 3 | 0 | 0.115290 |
| sep-horizontal | none | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 거리 1 m | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 거리 1.5 m | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 거리 2 m | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 거리 3 m | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 3%D+0.1 | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 3%D+0.2 | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 3%D+0.3 | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 5%D+0.1 | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 5%D+0.2 | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 5%D+0.3 | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 10%D+0.1 | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 10%D+0.2 | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 10%D+0.3 | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | 거리 1 ∧ 5%D+0.3 | 0 | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-wallclean | none | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 거리 1 m | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 거리 1.5 m | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 거리 2 m | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 거리 3 m | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 3%D+0.1 | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 3%D+0.2 | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 3%D+0.3 | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 5%D+0.1 | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 5%D+0.2 | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 5%D+0.3 | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 10%D+0.1 | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 10%D+0.2 | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 10%D+0.3 | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | 거리 1 ∧ 5%D+0.3 | 0 | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallfullclean | none | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 거리 1 m | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 거리 1.5 m | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 거리 2 m | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 거리 3 m | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 3%D+0.1 | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 3%D+0.2 | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 3%D+0.3 | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 5%D+0.1 | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 5%D+0.2 | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 5%D+0.3 | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 10%D+0.1 | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 10%D+0.2 | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 10%D+0.3 | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | 거리 1 ∧ 5%D+0.3 | 0 | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-boundary | none | 1 | 0 | 1 | 1.025460 | 1 | 34 | 2.240657 |
| sep-boundary | 거리 1 m | 0 | 1 | 1 | 0.067141 | 4 | 0 | 0.116287 |
| sep-boundary | 거리 1.5 m | 0 | 1 | 1 | 0.068784 | 5 | 0 | 0.116287 |
| sep-boundary | 거리 2 m | 0 | 1 | 1 | 0.069781 | 3 | 0 | 0.116287 |
| sep-boundary | 거리 3 m | 1 | 0 | 1 | 1.025460 | 1 | 34 | 2.240657 |
| sep-boundary | 3%D+0.1 | 0 | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | 3%D+0.2 | 0 | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | 3%D+0.3 | 0 | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | 5%D+0.1 | 0 | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | 5%D+0.2 | 0 | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | 5%D+0.3 | 0 | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | 10%D+0.1 | 0 | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | 10%D+0.2 | 0 | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | 10%D+0.3 | 0 | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | 거리 1 ∧ 5%D+0.3 | 0 | 1 | 1 | 0.067141 | 4 | 0 | 0.116287 |
| sep-colored | none | 1 | 0 | 1 | 0.817939 | 2 | 24 | 2.823264 |
| sep-colored | 거리 1 m | 0 | 1 | 1 | 0.046552 | 3 | 0 | 0.082924 |
| sep-colored | 거리 1.5 m | 1 | 1 | 1 | 0.413020 | 1 | 18 | 0.936890 |
| sep-colored | 거리 2 m | 0 | 1 | 1 | 0.043537 | 1 | 0 | 0.082954 |
| sep-colored | 거리 3 m | 1 | 0 | 1 | 0.894569 | 3 | 30 | 2.675667 |
| sep-colored | 3%D+0.1 | 0 | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | 3%D+0.2 | 0 | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | 3%D+0.3 | 0 | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | 5%D+0.1 | 0 | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | 5%D+0.2 | 0 | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | 5%D+0.3 | 0 | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | 10%D+0.1 | 0 | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | 10%D+0.2 | 0 | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | 10%D+0.3 | 0 | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | 거리 1 ∧ 5%D+0.3 | 0 | 1 | 1 | 0.046552 | 3 | 0 | 0.082924 |
| sep-westspine | none | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 거리 1 m | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 거리 1.5 m | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 거리 2 m | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 거리 3 m | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 3%D+0.1 | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 3%D+0.2 | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 3%D+0.3 | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 5%D+0.1 | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 5%D+0.2 | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 5%D+0.3 | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 10%D+0.1 | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 10%D+0.2 | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 10%D+0.3 | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | 거리 1 ∧ 5%D+0.3 | 0 | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-wallboth | none | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 거리 1 m | 0 | 0 | 0 | 0.033477 | 1 | 0 | 0.067612 |
| sep-wallboth | 거리 1.5 m | 0 | 0 | 0 | 0.033477 | 1 | 0 | 0.067612 |
| sep-wallboth | 거리 2 m | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 거리 3 m | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 3%D+0.1 | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 3%D+0.2 | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 3%D+0.3 | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 5%D+0.1 | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 5%D+0.2 | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 5%D+0.3 | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 10%D+0.1 | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 10%D+0.2 | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 10%D+0.3 | 0 | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | 거리 1 ∧ 5%D+0.3 | 0 | 0 | 0 | 0.033477 | 1 | 0 | 0.067612 |

## 새 graph에서 드러난 차이

거리 2.0 m는 첫 오류를 막았지만 새 `274→1745`를 1187.394 s에 삽입했다.
그 상대 XY 오차는 3.572812 m, 최종 map p95는 1.630722 m다.
9월의 2 m 인과 토글이나 고정 후보 재판정을 최종 해결로 채택할 수 없는 직접 근거다.

거리 1.5 m도 G02에서는 오류 0이지만 sep-colored에서 새 `133→265`를
948.799999991 s에 삽입했다. 상대 XY 오차는 1.422172 m다.
따라서 G02 한 번의 결과만으로 threshold를 정하면 이 9월 반례를 놓친다.
전체 10개에서 오류 수를 우선하면 거리 단독 최선은 1.0 m다.

3%D+.1 m와 3%D+.2 m는 큰 alias 대신 새 `236→313`을 227.002 s에 삽입했다.
상대 XY 오차 0.135869 m로 기존 0.10 m 정상 기준을 넘으므로 오류 1개로 센다.
online pose max는 0.113926 m이고 큰 지도 급합체와는 크기가 다르다.
이 새 오류를 숨기기 위해 기존 정상 기준을 완화하지 않았다.

## 실물 drift 위험과 허용량

모든 gate는 정상 loop에 필요한 보정이 경계를 넘으면 정상 closure를 거부한다.
실물 Go2 odometry drift를 측정한 자료는 이 비교에 없다. simulation의 낮은 drift를
실물 보증으로 일반화하지 않는다. 아래 값은 수학적 XY 허용량이며 yaw drift 보증이 아니다.

거리 gate는 같은 위치로 복귀한 정상 loop에 대해 odometry drift ≤X를 허용한다.
실제 두 scan 위치가 r m 떨어져도 정당한 overlapping closure일 수 있으며, 이때
보수적인 drift 여유는 X-r이다. X<r이면 drift가 없어도 정상 loop를 잃을 수 있다.
보정 gate는 마지막 유지 loop 이후 D에 대해 kD+c m를 허용한다.
조합 gate는 두 조건의 교집합이므로 각각의 실물 위험을 모두 갖는다.
보정량은 sensor pose 차이이며 실측 odometry 불확실성 자체가 아니다.
다른 loop가 유지되어 D를 reset했다고 오래된 linked anchor의 불확실성까지 사라지는 것은 아니다.
none은 추가 gate bound가 없으며 기존 false alias 위험을 유지한다.

| gate | D=10 m 허용 m | D=50 m 허용 m | D=100 m 허용 m |
| --- | ---: | ---: | ---: |
| none의 추가 제한 | 없음 | 없음 | 없음 |
| 거리 1 m | 1.00 | 1.00 | 1.00 |
| 거리 1.5 m | 1.50 | 1.50 | 1.50 |
| 거리 2 m | 2.00 | 2.00 | 2.00 |
| 거리 3 m | 3.00 | 3.00 | 3.00 |
| 3%D+0.1 | 0.40 | 1.60 | 3.10 |
| 3%D+0.2 | 0.50 | 1.70 | 3.20 |
| 3%D+0.3 | 0.60 | 1.80 | 3.30 |
| 5%D+0.1 | 0.60 | 2.60 | 5.10 |
| 5%D+0.2 | 0.70 | 2.70 | 5.20 |
| 5%D+0.3 | 0.80 | 2.80 | 5.30 |
| 10%D+0.1 | 1.10 | 5.10 | 10.10 |
| 10%D+0.2 | 1.20 | 5.20 | 10.20 |
| 10%D+0.3 | 1.30 | 5.30 | 10.30 |
| 거리 1 m ∧ 5%D+0.3 근사 | 0.80 | 1.00 | 1.00 |

조합 행은 동일 위치의 정상 closure에서 sensor 보정량이 상대 odometry drift와
같다는 근사 아래 두 cap의 최솟값이다. 실제로는 원본 odometry와 현재 sensor pose라는
서로 다른 상태를 검사하므로 이 값이 실물의 통계적 drift 보증은 아니다.

## 검증 범위

- 단위 계약 6개와 native 실제 엔진 회귀를 정식 ament pytest에 등록했다.
- native 회귀는 none/거리 1.5 m 두 반복/3%D+.1/10%D+.3의 첫 350 scan에서 실제 삽입, D·보정량, downstream pose 변화와 반복 edge·factor·pose·PGM byte 일치를 검사한다. 반복 범위에는 정상 244→328의 실제 최적화가 포함된다.
- 원본·수정 SHA256 구분, 상대 측정 정답/오답 대조, 정상 pair 부재의 의미, 재현 실패 차단을 검사했다.
- native build exit 0, 새 engine-recipe prefix에서 정식 build.sh 전체 실행 exit 0, patch reverse-apply check, ament 구성과 wall_hug_counterfactual 등록 확인은 통과했다. 등록된 CTest wrapper도 96.47 s에 exit 0으로 전체 파일을 실행했다.
- 기존 replay·지도 검사와 native/정상 pair 검사 35개, 나머지 계약 4개, retained arc 음성 대조 1개로 중복 제거한 40개가 통과했다. 실제 최적화를 포함한 최종 350 scan native 회귀는 96.59 s에 통과했다.
- baseline 10개의 원본 scan 시각은 모두 엄격히 증가한다. baseline 10개와 sweep 130개·조합 10개 모두에서 원본 arc와 실제 새 loop edge를 대조한 D reset 검사는 통과했다.
- 전체 colcon·hosted CI·새 GUI·실물 동작은 이번 검증 범위가 아니다.

## 현재 graph motion gate·robust back-end 후속 비교

simulation 완료를 우선하는 후속 승인 범위다. 공유 SLAM install·mapping YAML은
유지하고 `engine-motion-v2` 전용 prefix에서 raw scan을 재매칭·재최적화했다.
수치 산출물은 `.omo/evidence/loop-gate-replay/motion-robust/`에 보존한다.
원래 10개와 추가 두 개를 분리해 집계한다.

### 기존 판별 분석과 추가 입력

로컬 지식 저장소의
`records/experiments/go2_local_navigation_slam_loop_closure_discriminator_20260926.md`와
9월 21일 두 event의 `analysis.md`를 먼저 읽었다. 정상 3·false 3개는 chain,
coarse/fine response 범위가 겹치고, false `187→256`의 coarse variance
0.00186446은 정상 최솟값보다 작았다. ROS variance parameter .22에서도
잘못된 장소를 작은 covariance로 정합해 지도가 합쳐졌다는 결론이다.
과거 다섯 사례의 matcher-odometry 오차는 정상 최대 .152275 m와 false 최소
3.624381 m로 갈렸지만 여섯 번째의 동시각 값과 실물 drift는 미확인이었다.
따라서 더 낮은 variance·response·chain threshold를 공통 해법으로 승격하지 않았다.

이번 motion gate는 당시 제안한 graph-path·pose consistency를 실제 새 graph에서
계측한다. 기존 matcher threshold는 바꾸지 않는다. 이것은 simulation odometry를
활용한 별도 증거이며 과거의 `no_verified_real-go2_discriminator`와 모순되지 않는다.

두 사건 모두 raw range·laser 정의·원본 odometry를 가진 `.data`가 남아 있다.
현재 archive 위치는 `verification/archive-index.md`의 대응을 따르며 manifest에
원본 SHA256을 추가했다.

| 추가 dataset | archive event | data SHA256 |
| --- | --- | --- |
| sep21-1800 | `slam_loop_debug_live_20260921_1800/event_20260921_180820/posegraph.data` | `3b425fb2c91ddbdde777d97de3664c9b511a3c162ab6b38df27bae4306baa157` |
| sep21-variance022 | `slam_loop_debug_live_20260921_variance_022/event_20260921_2007/posegraph.data` | `85cdbc3d0281053368c4991d114e74368b8188fe63bf7cc728561816d1e07545` |

archive root는 `/home/bi-agx1/go2_projects/archive/go2_local_navigation/verification_untracked_20260921era/verification/`다.
1800 event의 기록된 world hash는 기존 world와 일치한다. .22 event의
`runtime_inputs.sha256`에는 별도 world hash가 없어 같은 당시 world였다는
실험 조건을 사용하며 byte 동일성을 독립 증명하지 않는다. truth는 직렬화된 simulation
odometry이며 독립 physics qpos가 아니다. 과거 .22 실행의 raw stream도 이번에는
동일 sigma 3.0으로 재생하므로 그 과거 설정·전체 edge list의 정확 재현으로 부르지 않는다.
사건 snapshot의 `/odom`을 closure timestamp의 odometry로 대체하지 않는다.

추가 두 none 재생은 각각 503/262 scan을 처리했고 과거 `40→97`, `173→234`,
`237→294`, `137→411`, `368→475`와 `187→256`을 같은 시각·같은 상대 factor로
재현했다. 다만 과거 사건의 시각적 정상/false 라벨을 현재 상대 측정 라벨로 복사하지 않았다.
SHA256을 함께 고정한 **원래 저장 graph**에서도 `237→294`의 상대 XY 오차는
3.584252 m, `187→256`은 3.597425 m인 반면 `368→475`는 .046382 m,
yaw .483319°로 현재 기준의 정상이다. 원래 graph와 새 none의 이 상대 측정값은 일치한다.
근거는 `sep21-*-original-factors.jsonl`과 `historical-relative-classification.json`이다.

과거 `475`의 matcher-odometry 절대 차이 3.624381 m에는 앞선 false closure로
이미 이동한 map frame의 누적 효과가 섞였다. 이를 새 false **상대 constraint**의
오차와 동일하게 셀 수 없다. 그래서 당시 정상 3/false 3 사건과 이번 정상 4/오류 2인
그 여섯 factor 분류를 구분한다. 과거 기록은 그대로 보존하며 .22가 실제 false
`187→256`을 통과시켰다는 반례와 단일 variance 분리 실패 결론은 유지한다.
원래 LinkInfo 생성 시 `368`의 y는 3.882036 m 대 odometry .253287 m,
`475`의 y는 5.551895 m 대 odometry 1.927577 m다. 두 끝점 모두
약 +3.63 m 이동해 있었으므로 이 상대 edge는 그 공통 이동을 새로 발생시킨 것이 아니다.

### 구현과 비용

기준 (a)는 마지막 실제 유지 loop부터 원래 odometry arc를 재는 기존 `5%D+.2 m`다.
(b)는 현재 corrected base pose에서 연속 scan chain과 실제 수락 loop edge로
무방향 graph를 만들고 linked anchor→query의 최단거리 D를 사용한다.
가중치는 **현재 pose 사이 XY 길이**다. 새 후보 edge는 수락 전이므로 포함하지 않고,
일반 local proximity link도 odometry-chain/accepted-loop graph에 넣지 않는다.
앞선 loop 최적화가 과거 node를 옮기면 다음 후보에서 모든 길이를 다시 계산한다.

연속 chain에서 branch가 생기는 곳은 유지 loop 끝점뿐이므로 anchor·query와
그 끝점 사이를 현재 prefix arc로 정확히 압축한다. Dijkstra는 이 작은 graph에서
수행한다. V개 scan과 L개 유지 loop의 후보 한 개 비용은 평균
O(V + L log L), 공간 O(V + L)이며 full graph all-pairs나 오래된 거리 cache를 쓰지 않는다.
decision에는 압축 graph 전체를 남겨 별도 Python Dijkstra로 D와 수락식을 검증했다.
예를 들어 G02 k=.03/a=.5의 `667→999` 후보는 chain-only 42.815127 m에서
이미 수락한 `771→864`를 지나 33.000394 m로 줄어든다.
경로 길이는 `667→771` 13.736107 + loop 2.142191 + `864→999` 17.122096 m다.
다음 `667→1000`에서는 기존 최적화가 pose를 바꿔 D도 32.778047 m로 갱신된다.

네 motion grid × 12개, 총 48회에서 후보 330개를 계측했다. 경로 snapshot
작성·Dijkstra의 합은 155,128.765 µs, 후보 평균 .470087 ms, 최대 1.837510 ms였다.
이 시간은 decision JSON 직렬화와 matcher/Ceres 실행을 제외한다.

요청에 최종 명시된 선형 lever-arm 식을 적용했다.
`theta_max=(a*D+b)*pi/180`, `XY≤k*D+.2+D*theta_max`,
`abs(wrap(dyaw))≤theta_max`다. XY와 dyaw는 fine matcher가 제안한 sensor pose와
현재 sensor pose의 차이다. sin(theta) 대신 D*theta를 사용하며, k=.03/.05,
a=.5/1 deg/m, b=2 deg 네 조합을 모두 비교한다. 통계적 covariance bound나
실제 heading drift 측정값으로 해석하지 않는다.

(c)는 설치된 `ros-humble-slam-toolbox 2.6.10-1jammy.20260909.214512`에서
`ceres_loss_function`을 확인했다. `/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_sync.yaml`은
`None`을 제공하며 설치 plugin에도 parameter·HuberLoss·CauchyLoss symbol이 있다.
plugin SHA256은 `10a3bb177e636dd0447078efa32e3e09d81381f422facd2a9bbca1040f3f76cc`다.
같은 2.6.10 source `solvers/ceres_solver.cpp:40-60,303-347`은
HuberLoss(.7)/CauchyLoss(.7)를 선택하고 covariance로 whiten한 XY/yaw residual
block에 적용한다. **모든 chain·local·loop constraint**에 적용되며 false loop
자체를 거부하는 front-end gate는 아니다. 각 replay process의 NodeOptions로만
loss를 override했고 공유 parameter 파일은 수정하지 않았다.

G02 첫 alias `140→275`에서 원래 retained arc는 35.852407 m, 새 current-graph
D는 17.449889 m다. 가장 엄격한 k=.03/a=.5도 XY 허용량이 3.989864 m로
실제 보정 3.560014 m보다 크다. yaw 보정은 -.799849°인데 허용은 10.724944°다.
따라서 네 조합 모두 첫 대형 false loop를 통과시킨다. 두 평행 통로의 병진 alias는
yaw가 작을 수 있고, `D*(aD+b)`가 D에 대해 이차로 커지므로 heading lever-arm
항을 더하는 것만으로 이 사례를 차단할 수 없다는 직접적인 counterexample다.

### 오류 loop 0이어도 남는 이중벽의 원인

`counterfactual_profiles.py`가 baseline raw range를 그대로 두고 원래 odometry와
최종 graph의 sensor pose로 각각 endpoint를 배치했다. 이 세 dataset은
실제 새 loop edge가 0이고 movement filter도 0이다. 기존 edge에 대한 수락 callback도
최적화를 호출할 수 있으므로 pre-loop 관찰 pose를 최종 pose로 간주하지 않았다.
horizontal/fullclean은 실제 차이가 0이고 wallboth는 최대 .007371 m 차이가 있다.
각 wall의 기존 `doubled_runs_ge_020m`과 진단 스크립트의 수가 정확히 같음을
assert했다. 아래 u는 벽 중심에서 긴 축 방향 거리, normal은 중심선 법선 거리다.
실제 `north_band_west`·`south_band_mid`의 물리적 두 면은 normal ±.060 m다.
원시 분포·시각 사분위·모든 histogram과 점유 구간은 `profiles-final/*.json`에 있다.

| dataset | 기존 doubled / loop | 대표 wall·u 구간 m | 대표 map normal 점유 구간 m | 주원인 |
| --- | ---: | --- | --- | --- |
| sep-horizontal | 5 / 0 | north_band_west, 1.875~2.875 | -.2725~-.2350, -.1725~-.0850 | loop 없는 순차 scan-matcher drift |
| sep-wallfullclean | 3 / 0 | south_band_mid, -1.125~.575 | -.2725~-.1350, -.0225~.0150 | loop 없는 순차 scan-matcher drift |
| sep-wallboth | 1 / 0 | south_band_mid, -1.125~2.275 | -.0350~.0025, .0650~.1025 | 두 실제 면·5 cm raster·중심선 교차를 doubled로 세는 metric artifact |

horizontal의 같은 북쪽 벽 raw endpoint를 odometry로 배치하면 normal p5/p50/p95는
-.0672/-.0593/-.0477 m다. scan matching으로 배치하면 -.2995/-.1712/-.1436 m다.
초반 97.830~100.628 s의 matched median -.1557 m가 후반
334.324~336.924 s에는 -.2892 m로 바뀌지만 odometry median은
-.0545→-.0613 m다. 같은 쪽 실제 면이 시간에 따라 .13 m 이상 이동한다.
최대 online pose 오차도 .467467 m이며 이 벽을 본 scan의 sensor XY 오차가
같은 값까지 커진다. 5 cm raster만으로 .2 m 이상 누적된 이동을 설명할 수 없다.

fullclean의 남쪽 벽은 odometry endpoint의 p5/p50/p95가
-.0782/.0574/.0722 m로 실제 두 면에 가깝다. matched 배치에서는
-.3477/-.2216/-.0165 m다. 35.650~40.348 s median -.0367 m에서
145.944~174.344 s median -.2518 m로 이동하며 online pose max는
.474472 m다. west_spine_lower와 upper에도 각각 .50 m, 1.00 m 길이의
doubled run이 남는다. false loop가 없어도 순차 정합의 frame 배치가 틀어지는
증거다. 두 dataset의 ghost 25개씩과 p95 .343819/.297898 m도 실제 열화를 뒷받침한다.

반면 wallboth는 ghost 0, p95 .032033 m, pose max .069841 m다. raw endpoint
matched p5/p50/p95는 -.0602/.0581/.0684 m로 두 실제 면을 유지한다.
map의 음수 쪽 5 cm cell 구간이 -.0350~+.0025 m로 **중심선을 조금 넘기 때문에**
현재 지표의 `sides` 구성은 그 구간을 양쪽 면 목록에 모두 넣는다. 양수 쪽
.0650~.1025 m 구간과 center 간격이 .10 m를 넘으면서 한 쪽에 두 벽이 있다고 센다.
대표 단면의 두 center는 -.01625/.08375 m이며 실제 계산 차이는
`.10000000000000009` m다. .10 m strict 경계의 부동소수점 효과도 판정에 들어간다.
양면 관측과 작은 pose/raster 이동이 3.40 m 길이의 doubled run으로 증폭된 사례다.
새 가짜 벽이 raw scan에 나타난 근거는 없으며 raytracing만의 결함으로 분류하지 않는다.

따라서 잔여 doubled 수 전부를 같은 원인으로 보거나 0 wrong loop를 지도 합격과
동일시할 수 없다. horizontal/fullclean의 큰 잔차는 실제 순차 matcher drift가
주도하고 wallboth의 1개는 현재 지표의 면 분류 효과가 주도한다. 이번 범위에서는
matcher·map resolution·raytracing·지표를 수정하거나 기존 수용 threshold를 완화하지 않았다.

### Simulation 완료를 위한 권고

**시뮬레이션의 false-loop 차단 후보는 거리 1.0 m + 기존 None backend를 유지한다.**
이번 새 식은 heading 불확실성을 허용하는 모델이지 반복 통로 alias를 가르는
독립 관측이 아니다. 가장 엄격한 조합도 첫 3.55 m false 상대 측정을 보존한다.
Cauchy는 큰 잘못된 constraint의 지도 영향을 줄일 수 있지만 삽입 자체를 막지 않고,
Huber는 G02 지도·online pose를 오히려 악화시켰다.

권고의 추가 두 반례도 직접 재생했다. 이는 `--combined 1 -1 0`으로 기존
거리 1 m만 활성화한 추가 대조이며 motion+robust 조합과 구분한다.

| dataset | gate | 오류 (XY>1 m) | 기준 정상 pair 부재 | 새 graph 정상 총 | p95 m | doubled | ghost | online pose max m |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| sep21-1800 | 거리 1 m | 0 (0) | 4 | 5 | .050226 | 3 | 0 | .229723 |
| sep21-variance022 | 거리 1 m | 0 (0) | 1 | 1 | .050435 | 8 | 0 | .121111 |

원래 10개와 합치면 거리 1 m는 엄격한 오류 0, 정상 pair 부재 17, 새 정상 총 21,
p95 max .537505 m, doubled 합 42, ghost 합 97, online pose max .724970 m다.
그중 원래 10개는 앞선 검증된 실행을 사용했고 추가 두 개만 새로 실행했다.
5개 정상 pair 부재를 정상 closure 다섯 기회가 실제 사라졌다는 뜻으로 읽지 않는다.
추가 두 실행에서 새 정상 총은 6개다.

이는 false-loop 대응의 simulation 후보 선택이며 **simulation 지도 수용 전체의
완료 판정은 아니다**. 별도 과제는 loop가 없는 자료의 순차 matcher drift와
기존 지도 지표의 양면/raster 경계다. 원래 map 합격 기준을 유지하면 잔차가 남고,
이번 오프라인 비교만으로 새 GUI·AMCL/Nav2 주행 합격을 주장할 수 없다.
실물에서는 실제 odometry drift와 overlapping closure의 정당한 노드 간 거리가
1 m를 넘을 수 있으므로 같은 절대 gate를 기본값으로 옮기지 않는다.


### 최종 선택과 원래 10개 합계

12개 baseline·84개 (a)/(b)/(c)·12개 (d), 합계 108개를 새 prefix에서 끝냈다.
거리 1 m 추가 대조 두 개는 별도라 새 전체 raw 재생은 110회다.
원래 10개에서 엄격한 오류 합, G02 online max, G02 p95, 새 정상 총 순으로
선택했으며 동률이면 큰 k/a를 택했다. 추가 두 개는 선택에 쓰지 않았다.
(b)는 3%/.5°/m, (c)는 CauchyLoss가 최선이고 (d)는 그 조합이다.
기계 집계는 `motion-comparison.json`, 선택은 `motion-selection.json`이다.

최종 검증은 다음과 같다.

- 격리 엔진 build exit 0 및 해당 prefix의 Karto library link 확인.
- 정식 CTest `wall_hug_counterfactual`: 298.18초, pytest 16개, 실패 0·skip 0.
- 네 motion·두 loss·선택 조합 각각 native 두 반복의 edge/factor/online/final/decision/PGM byte 동일.
- 기존 geometry·recorded replay 회귀 33개 통과.
- SHA256 17개와 12개 raw stream의 유한 odometry·엄격 증가 시각 확인.
- 본 비교·추가 거리 대조 110회 모두 input 수와 accepted scan 수 일치.
- 기존 v1 엔진으로 새 CLI의 legacy 8-인자 baseline 실행 통과.
- 구문 검사·`git diff --check` 통과. 전체 colcon·hosted CI·GUI·실시간 실행은 수행하지 않았다.

잔여 이중벽을 일관되게 줄인 새 motion gate는 없다. 원래 10개에서 Cauchy의
doubled 20은 기준식 21보다 1개 적은 정도이며 G02 자체는 1→3으로 늘었다.
12개 합계에서는 Cauchy 22 대 기준식 30으로 8개(26.7%) 줄지만, 그중 7개는
추가 두 dataset의 9→2에서 나온다. 엄격한 오류는 1→11로 늘고 XY>1 m 오류도
10개다. 조합은 doubled 28·오류 8로 Cauchy 단독보다 이점이 없다.
특히 horizontal/fullclean/wallboth의 5/3/1은 **모든 variant에서 그대로**다.
큰 잔차의 원인이 loop 없는 순차 정합과 지표 경계이므로 robust back-end만으로
그 잔차를 해결했다고 볼 수 없다.

| gate | 오류 (XY>1 m) | 기준 정상 pair 부재 | 새 graph 정상 총 | p95 max m | doubled 합 | ghost 합 | online pose max m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 10 (6) | 0 | 13 | 1.243856 | 25 | 300 | 5.027775 |
| (a) 기존 5%D+.2 | 1 (0) | 9 | 14 | 0.537505 | 21 | 97 | 0.724970 |
| (b) k=3%, a=.5°/m | 6 (2) | 0 | 14 | 1.061744 | 24 | 197 | 5.027775 |
| (b) k=3%, a=1°/m | 9 (5) | 0 | 14 | 1.061744 | 23 | 280 | 5.027775 |
| (b) k=5%, a=.5°/m | 7 (3) | 0 | 14 | 1.061744 | 26 | 221 | 5.027775 |
| (b) k=5%, a=1°/m | 9 (5) | 0 | 14 | 1.061744 | 23 | 280 | 5.027775 |
| (c) HuberLoss | 10 (8) | 9 | 14 | 2.036963 | 30 | 272 | 7.813917 |
| (c) CauchyLoss | 6 (5) | 9 | 13 | 0.537505 | 20 | 97 | 0.724970 |
| (d) 3%/.5 + Cauchy | 6 (5) | 8 | 13 | 0.537505 | 23 | 97 | 0.724970 |

### G02 6차 별도 행

| dataset | gate | 오류 (XY>1 m) | 기준 정상 pair 부재 | 새 graph 정상 총 | p95 m | doubled | ghost | online pose max m |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| g02-6 | none | 5 (1) | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| g02-6 | (a) 기존 5%D+.2 | 1 (0) | 9 | 10 | 0.065271 | 1 | 0 | 0.113926 |
| g02-6 | (b) k=3%, a=.5°/m | 5 (1) | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| g02-6 | (b) k=3%, a=1°/m | 5 (1) | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| g02-6 | (b) k=5%, a=.5°/m | 5 (1) | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| g02-6 | (b) k=5%, a=1°/m | 5 (1) | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| g02-6 | (c) HuberLoss | 5 (3) | 9 | 10 | 2.036963 | 11 | 105 | 7.813917 |
| g02-6 | (c) CauchyLoss | 3 (2) | 9 | 9 | 0.067470 | 3 | 0 | 0.263116 |
| g02-6 | (d) 3%/.5 + Cauchy | 4 (3) | 8 | 9 | 0.069606 | 1 | 0 | 0.410903 |

### 추가 두 개를 포함한 12개 합계

| gate | 오류 (XY>1 m) | 기준 정상 pair 부재 | 새 graph 정상 총 | p95 max m | doubled 합 | ghost 합 | online pose max m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| none | 12 (8) | 0 | 18 | 1.584338 | 33 | 425 | 5.027775 |
| (a) 기존 5%D+.2 | 1 (0) | 11 | 23 | 0.537505 | 30 | 97 | 0.724970 |
| (b) k=3%, a=.5°/m | 8 (4) | 1 | 19 | 1.562504 | 41 | 259 | 5.027775 |
| (b) k=3%, a=1°/m | 11 (7) | 1 | 19 | 1.562504 | 40 | 342 | 5.027775 |
| (b) k=5%, a=.5°/m | 9 (5) | 1 | 19 | 1.562504 | 43 | 283 | 5.027775 |
| (b) k=5%, a=1°/m | 12 (8) | 1 | 19 | 1.562504 | 40 | 405 | 5.027775 |
| (c) HuberLoss | 12 (10) | 10 | 19 | 2.036963 | 39 | 314 | 7.813917 |
| (c) CauchyLoss | 11 (10) | 11 | 21 | 0.537505 | 22 | 97 | 0.724970 |
| (d) 3%/.5 + Cauchy | 8 (7) | 9 | 20 | 0.537505 | 28 | 97 | 0.724970 |

### dataset별 후속 108개 실행

아래는 none까지 포함한 9개 variant × 12개 dataset의 개별 결과다.
오류 크기·정상 pair 부재·새 정상 총의 정의는 앞선 비교와 동일하다.

| dataset | gate | 오류 (XY>1 m) | 기준 정상 pair 부재 | 새 graph 정상 총 | p95 m | doubled | ghost | online pose max m |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| g02-6 | none | 5 (1) | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| sep-cause | none | 1 (1) | 0 | 1 | 0.983311 | 1 | 25 | 1.890487 |
| sep-toggle | none | 2 (2) | 0 | 0 | 1.243856 | 2 | 45 | 3.746897 |
| sep-horizontal | none | 0 (0) | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-wallclean | none | 0 (0) | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallfullclean | none | 0 (0) | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-boundary | none | 1 (1) | 0 | 1 | 1.025460 | 1 | 34 | 2.240657 |
| sep-colored | none | 1 (1) | 0 | 1 | 0.817939 | 2 | 24 | 2.823264 |
| sep-westspine | none | 0 (0) | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-wallboth | none | 0 (0) | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep21-1800 | none | 1 (1) | 0 | 4 | 1.584338 | 2 | 62 | 3.584454 |
| sep21-variance022 | none | 1 (1) | 0 | 1 | 1.365162 | 6 | 63 | 3.353882 |
| g02-6 | (a) 기존 5%D+.2 | 1 (0) | 9 | 10 | 0.065271 | 1 | 0 | 0.113926 |
| g02-6 | (b) k=3%, a=.5°/m | 5 (1) | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| g02-6 | (b) k=3%, a=1°/m | 5 (1) | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| g02-6 | (b) k=5%, a=.5°/m | 5 (1) | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| g02-6 | (b) k=5%, a=1°/m | 5 (1) | 0 | 10 | 1.061744 | 9 | 75 | 5.027775 |
| g02-6 | (c) HuberLoss | 5 (3) | 9 | 10 | 2.036963 | 11 | 105 | 7.813917 |
| g02-6 | (c) CauchyLoss | 3 (2) | 9 | 9 | 0.067470 | 3 | 0 | 0.263116 |
| sep-cause | (a) 기존 5%D+.2 | 0 (0) | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | (b) k=3%, a=.5°/m | 0 (0) | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | (b) k=3%, a=1°/m | 1 (1) | 0 | 1 | 0.983311 | 1 | 25 | 1.890487 |
| sep-cause | (b) k=5%, a=.5°/m | 0 (0) | 0 | 1 | 0.087773 | 5 | 0 | 0.166456 |
| sep-cause | (b) k=5%, a=1°/m | 1 (1) | 0 | 1 | 0.983311 | 1 | 25 | 1.890487 |
| sep-cause | (c) HuberLoss | 1 (1) | 0 | 1 | 0.255036 | 1 | 12 | 0.506496 |
| sep-cause | (c) CauchyLoss | 1 (1) | 0 | 1 | 0.060290 | 2 | 0 | 0.166456 |
| sep-toggle | (a) 기존 5%D+.2 | 0 (0) | 0 | 1 | 0.036679 | 5 | 0 | 0.114857 |
| sep-toggle | (b) k=3%, a=.5°/m | 1 (1) | 0 | 1 | 0.422403 | 0 | 25 | 0.955205 |
| sep-toggle | (b) k=3%, a=1°/m | 1 (1) | 0 | 1 | 0.422403 | 0 | 25 | 0.955205 |
| sep-toggle | (b) k=5%, a=.5°/m | 1 (1) | 0 | 1 | 0.422403 | 0 | 25 | 0.955205 |
| sep-toggle | (b) k=5%, a=1°/m | 1 (1) | 0 | 1 | 0.422403 | 0 | 25 | 0.955205 |
| sep-toggle | (c) HuberLoss | 2 (2) | 0 | 1 | 0.350198 | 2 | 22 | 0.974260 |
| sep-toggle | (c) CauchyLoss | 1 (1) | 0 | 1 | 0.061017 | 0 | 0 | 0.107754 |
| sep-horizontal | (a) 기존 5%D+.2 | 0 (0) | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | (b) k=3%, a=.5°/m | 0 (0) | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | (b) k=3%, a=1°/m | 0 (0) | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | (b) k=5%, a=.5°/m | 0 (0) | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | (b) k=5%, a=1°/m | 0 (0) | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | (c) HuberLoss | 0 (0) | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-horizontal | (c) CauchyLoss | 0 (0) | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-wallclean | (a) 기존 5%D+.2 | 0 (0) | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | (b) k=3%, a=.5°/m | 0 (0) | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | (b) k=3%, a=1°/m | 0 (0) | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | (b) k=5%, a=.5°/m | 0 (0) | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | (b) k=5%, a=1°/m | 0 (0) | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | (c) HuberLoss | 0 (0) | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallclean | (c) CauchyLoss | 0 (0) | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallfullclean | (a) 기존 5%D+.2 | 0 (0) | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | (b) k=3%, a=.5°/m | 0 (0) | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | (b) k=3%, a=1°/m | 0 (0) | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | (b) k=5%, a=.5°/m | 0 (0) | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | (b) k=5%, a=1°/m | 0 (0) | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | (c) HuberLoss | 0 (0) | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-wallfullclean | (c) CauchyLoss | 0 (0) | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-boundary | (a) 기존 5%D+.2 | 0 (0) | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | (b) k=3%, a=.5°/m | 0 (0) | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | (b) k=3%, a=1°/m | 1 (1) | 0 | 1 | 1.025460 | 1 | 34 | 2.240657 |
| sep-boundary | (b) k=5%, a=.5°/m | 0 (0) | 0 | 1 | 0.054399 | 0 | 0 | 0.116287 |
| sep-boundary | (b) k=5%, a=1°/m | 1 (1) | 0 | 1 | 1.025460 | 1 | 34 | 2.240657 |
| sep-boundary | (c) HuberLoss | 1 (1) | 0 | 1 | 0.490746 | 0 | 17 | 0.848228 |
| sep-boundary | (c) CauchyLoss | 1 (1) | 0 | 1 | 0.068992 | 2 | 0 | 0.156704 |
| sep-colored | (a) 기존 5%D+.2 | 0 (0) | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | (b) k=3%, a=.5°/m | 0 (0) | 0 | 1 | 0.039731 | 0 | 0 | 0.083966 |
| sep-colored | (b) k=3%, a=1°/m | 1 (1) | 0 | 1 | 0.817939 | 2 | 24 | 2.823264 |
| sep-colored | (b) k=5%, a=.5°/m | 1 (1) | 0 | 1 | 0.817939 | 2 | 24 | 2.823264 |
| sep-colored | (b) k=5%, a=1°/m | 1 (1) | 0 | 1 | 0.817939 | 2 | 24 | 2.823264 |
| sep-colored | (c) HuberLoss | 1 (1) | 0 | 1 | 0.392433 | 6 | 19 | 1.052830 |
| sep-colored | (c) CauchyLoss | 0 (0) | 0 | 1 | 0.041676 | 3 | 0 | 0.084267 |
| sep-westspine | (a) 기존 5%D+.2 | 0 (0) | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | (b) k=3%, a=.5°/m | 0 (0) | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | (b) k=3%, a=1°/m | 0 (0) | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | (b) k=5%, a=.5°/m | 0 (0) | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | (b) k=5%, a=1°/m | 0 (0) | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | (c) HuberLoss | 0 (0) | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-westspine | (c) CauchyLoss | 0 (0) | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-wallboth | (a) 기존 5%D+.2 | 0 (0) | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | (b) k=3%, a=.5°/m | 0 (0) | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | (b) k=3%, a=1°/m | 0 (0) | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | (b) k=5%, a=.5°/m | 0 (0) | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | (b) k=5%, a=1°/m | 0 (0) | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | (c) HuberLoss | 0 (0) | 0 | 0 | 0.032033 | 1 | 0 | 0.069841 |
| sep-wallboth | (c) CauchyLoss | 0 (0) | 0 | 0 | 0.029328 | 1 | 0 | 0.067095 |
| sep21-1800 | (a) 기존 5%D+.2 | 0 (0) | 2 | 8 | 0.041234 | 3 | 0 | 0.315910 |
| sep21-1800 | (b) k=3%, a=.5°/m | 2 (2) | 1 | 4 | 1.562504 | 11 | 62 | 3.272390 |
| sep21-1800 | (b) k=3%, a=1°/m | 2 (2) | 1 | 4 | 1.562504 | 11 | 62 | 3.272390 |
| sep21-1800 | (b) k=5%, a=.5°/m | 2 (2) | 1 | 4 | 1.562504 | 11 | 62 | 3.272390 |
| sep21-1800 | (b) k=5%, a=1°/m | 2 (2) | 1 | 4 | 1.562504 | 11 | 62 | 3.272390 |
| sep21-1800 | (c) HuberLoss | 1 (1) | 1 | 4 | 1.449926 | 8 | 29 | 3.785099 |
| sep21-1800 | (c) CauchyLoss | 4 (4) | 2 | 7 | 0.058756 | 0 | 0 | 0.450950 |
| sep21-variance022 | (a) 기존 5%D+.2 | 0 (0) | 0 | 1 | 0.054436 | 6 | 0 | 0.096710 |
| sep21-variance022 | (b) k=3%, a=.5°/m | 0 (0) | 0 | 1 | 0.054436 | 6 | 0 | 0.096710 |
| sep21-variance022 | (b) k=3%, a=1°/m | 0 (0) | 0 | 1 | 0.054436 | 6 | 0 | 0.096710 |
| sep21-variance022 | (b) k=5%, a=.5°/m | 0 (0) | 0 | 1 | 0.054436 | 6 | 0 | 0.096710 |
| sep21-variance022 | (b) k=5%, a=1°/m | 1 (1) | 0 | 1 | 1.365162 | 6 | 63 | 3.353882 |
| sep21-variance022 | (c) HuberLoss | 1 (1) | 0 | 1 | 0.735228 | 1 | 13 | 3.510873 |
| sep21-variance022 | (c) CauchyLoss | 1 (1) | 0 | 1 | 0.073776 | 2 | 0 | 0.133653 |
| g02-6 | (d) 3%/.5 + Cauchy | 4 (3) | 8 | 9 | 0.069606 | 1 | 0 | 0.410903 |
| sep-cause | (d) 3%/.5 + Cauchy | 0 (0) | 0 | 1 | 0.088072 | 3 | 0 | 0.166456 |
| sep-toggle | (d) 3%/.5 + Cauchy | 1 (1) | 0 | 1 | 0.080270 | 4 | 0 | 0.153161 |
| sep-horizontal | (d) 3%/.5 + Cauchy | 0 (0) | 0 | 0 | 0.343819 | 5 | 25 | 0.467467 |
| sep-wallclean | (d) 3%/.5 + Cauchy | 0 (0) | 0 | 0 | 0.537505 | 1 | 29 | 0.672665 |
| sep-wallfullclean | (d) 3%/.5 + Cauchy | 0 (0) | 0 | 0 | 0.297898 | 3 | 25 | 0.474472 |
| sep-boundary | (d) 3%/.5 + Cauchy | 1 (1) | 0 | 1 | 0.068992 | 2 | 0 | 0.156704 |
| sep-colored | (d) 3%/.5 + Cauchy | 0 (0) | 0 | 1 | 0.041676 | 3 | 0 | 0.084267 |
| sep-westspine | (d) 3%/.5 + Cauchy | 0 (0) | 0 | 0 | 0.497290 | 0 | 18 | 0.724970 |
| sep-wallboth | (d) 3%/.5 + Cauchy | 0 (0) | 0 | 0 | 0.029328 | 1 | 0 | 0.067095 |
| sep21-1800 | (d) 3%/.5 + Cauchy | 2 (2) | 1 | 6 | 0.054574 | 0 | 0 | 0.362397 |
| sep21-variance022 | (d) 3%/.5 + Cauchy | 0 (0) | 0 | 1 | 0.060201 | 5 | 0 | 0.133653 |
