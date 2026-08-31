# data

이 디렉터리는 센서 수용 검증용 rosbag과 실행 로그를 소스 패키지에서 분리해
보관한다. `.gitignore` 대상이므로 bag 원본은 Git에 포함하지 않고, 반복 참조할
경로·메시지 수·checksum은 저장소의 실행 record에 기록한다.

2026-08-26 정지 상태 검증 산출물은 다음과 같다.

- `bags/go2_stationary_raw_20260826_1829`: `/utlidar/cloud`와
  `/utlidar/robot_odom` source 원본
- `bags/go2_stationary_derived_20260826_1847`: `/odom`과
  `/perception/obstacle_candidates` replay 결과
- `runs/go2_stationary_perception_replay_20260826_1847`: runtime log
- `runs/go2_clean_teardown_replay_20260826_1858`: 종료 회귀 검증 log

정확한 수치와 checksum은
`records/experiments/go2_local_navigation_stationary_bag_perception_20260826.yaml`을
따른다.

2026-08-27 비동작 control·Nav2 산출물은 다음과 같다.

- `bags/go2_motion_dry_run_20260827`: candidate Twist 4개와 Sport Request preview 6개
- `bags/go2_nav2_controller_preview_20260827`: candidate, preview, local costmap,
  obstacle candidate, odometry와 path 총 10205개 message

정확한 topic별 수치와 checksum은 각각
`records/experiments/go2_local_navigation_motion_adapter_dry_run_20260827.yaml`,
`records/experiments/go2_local_navigation_nav2_non_actuating_preview_20260827.yaml`을
따른다.

같은 날 9단계 통합 비동작 preflight 산출물은 다음 경로에 있다.

- `runs/preflight/20260827_020502_stage9/result.json`: host·observer·teardown을 합친
  최종 46개 check 결과
- `runs/preflight/20260827_020502_stage9/observer.json`: ROS graph·TF·topic·gate 관찰
- `runs/preflight/20260827_020502_stage9/launch.log`: 전체 launch와 clean exit 로그
- `runs/preflight/20260827_020502_stage9/tegrastats.log`: 30초 AGX 자원 표본

최종 판정, 수치와 checksum은
`records/experiments/go2_local_navigation_integrated_preflight_20260827.yaml`을 따른다.

같은 날 10단계 정지 안정성 산출물은 다음 경로에 있다.

- `runs/preflight/20260827_150959_stage10_smoke`: 30초 smoke, 47 PASS
- `runs/preflight/20260827_151053_stage10_soak`: 30분 soak, 46 PASS·yaw drift 1 WARN

수신률·gap·drift·자원과 파일별 checksum은
`records/experiments/go2_local_navigation_stationary_soak_20260827.yaml`을 따른다.

12단계 준비용 read-only recorder QA 산출물은 다음 경로에 있다.

- `runs/trial_recorder/20260827_stage12_readonly_qa.json`: 종료 경계 실패를 보존한 첫 실행
- `runs/trial_recorder/20260827_stage12_readonly_qa_fixed.json`: 수정 후 exit 0 실행

두 artifact의 상태와 checksum은
`records/experiments/go2_local_navigation_trial_recorder_readonly_qa_20260827.yaml`을
따른다. 두 JSON의 `unverified` 상태는 실제 물리 trial이 수행되지 않았음을 뜻한다.

## DimOS 외부 replay custody

`external/dimos_go2_indoor/`는 pinned DimOS fixture의 로컬 전용 보관 경계다. 이
경로의 archive, 추출 MCAP, canonical bag과 실행 결과는 Git·release·재배포 대상이
아니다. repository의 Apache-2.0 license와 dataset 권리는 구분하며 현재 dataset
상태는 `dataset_license_unverified`다.

여기서 `external`은 외장 센서가 아니라 외부 출처 dataset이라는 뜻이다. canonical
short·full의 원본은 `source/go2_china_office_indoor.mcap`이다. topic·frame·rate와
pinned DimOS Go2 코드에 근거해 센서 계열을 `go2_built_in_l1_ulidar`로 강하게
추론하지만 hardware manifest가 없으므로 상태는 `unverified`다. 현재 프로젝트
로봇과 동일한 물리 calibration이라는 뜻도 아니다.

raw source는 `rt/utlidar/cloud`, `rt/utlidar/imu`, `rt/lowstate`,
`rt/sportmodestate`, `rt/utlidar/robot_odom`, `rt/frontvideo`, `control_log`,
`telemetry`의 8개 channel을 가진다. derived canonical short·full bag은 선택된
`/utlidar/cloud`와 `/utlidar/robot_odom` 두 topic만 가진다.

`source/recording_go2_mid360_2026-05-29_4-45pm-PST_corrected.db`는 같은 custody
경계에 보관된 Mid-360·Point-LIO fixture지만 canonical short·full 변환에는 사용하지
않았다. 파일이 같은 디렉터리에 있다는 사실을 데이터 계보로 해석하지 않는다.

- `source/`: hash와 size를 통과해 승격된 archive와 단일 raw MCAP
- `staging/`: bounded download와 secure extraction의 임시 파일
- `derived/`: short·full canonical rosbag2 MCAP
- `runs/`: acquisition·conversion 결과와 bag-info 근거

source identity와 제한값은 `src/go2_validation/config/external_replay_sources.yaml`, 사람이
확인할 provenance와 권리 경계는 지식 저장소의
`sources/repositories/related/dimos_go2_replay_source_card.md`를 따른다. 네트워크,
LFS 또는 최초 여유 공간 부족만 `deferred`이며, hash·size·tar member·CRC·schema·
CDR·count 불일치는 `conflict`다.

2026-08-27 canonical 변환과 ingress 결과는 다음 경로에 있다.

- `external/dimos_go2_indoor/runs/conversion.json`: source checksum, selected channel,
  short/full count와 tree checksum
- `external/dimos_go2_indoor/derived/short`: cloud `1843`, odometry `18026`
- `external/dimos_go2_indoor/derived/full`: cloud `17776`, odometry `173616`
- `runs/fault_acceptance/stage11.json`: Domain 61 fault 10개 PASS
- `runs/mapping_input/stage12-ingress.json`: 정지·external short ingress 모두 PASS

이 자료는 local-only software evidence다. 외부 dataset을 ground truth로 간주하거나
지도 정확도·실제 장애물 회피·목적지 도달 근거로 사용하지 않는다.

## Domain 63 SLAM mapping replay

Todo 12 mapping 하위 작업의 최종 local-only 산출물은 다음 경로에 있다.

- `runs/mapping/stage12.json`: stationary·external-full 최종 summary, 둘 다 PASS
- `runs/mapping/project_stationary/`: 정지 bag result·log와 occupancy·pose graph
- `runs/mapping/external_dynamic_full/`: 외부 full replay result·log와 occupancy·pose graph
- `runs/mapping.failed_20260827_221729/`: Humble lifecycle service 오계약 실패 근거
- `runs/mapping.failed_20260827_222603/`: 종료 직후 graph cache settle 실패 근거

정확한 stream count, source·replay checksum, artifact 크기·checksum과 종료 경계는
`records/experiments/go2_local_navigation_slam_mapping_replay_20260827.yaml`을 따른다.
최종 run은 residual node·process 0이지만 SIGINT 5초 뒤 SIGTERM으로 승격됐으므로
순수 SIGINT graceful teardown으로 표현하지 않는다. 생성된 지도는 저장·재로딩
검증용이며 지도 정확도·loop closure·localization의 합격 근거가 아니다.

## Domain 0 live mapping shadow

2026-08-31 Go2 ON·AGX 임시 배치·정지 조건의 local-only runtime artifact는 다음
경로에 있다.

- `runs/live_mapping/20260831_142854_todo12l_domain0_live_mapping/artifacts/`
  - `occupancy.pgm`, `occupancy.yaml`
  - `pose_graph.posegraph`, `pose_graph.data`

onboard 기본 `project_default/raw_single/onboard_observe`, `use_sim_time=false`에서
live `/scan`, `/odom`, `/map`, SLAM Toolbox 단독 `map → odom`, 저장·재로딩 service와
단일 SIGINT clean teardown을 확인했다. 반복 수치·checksum·warning은
`records/experiments/go2_local_navigation_domain0_live_mapping_shadow_20260831.yaml`을
따른다. 저장 occupancy와 재로딩 직후 live map의 크기가 달랐으므로 이 artifact는
연결성·service 검증용이며 지도 정확도나 저장 전후 픽셀 동일성의 합격 근거가 아니다.

## Domain 64 saved-map localization

2026-08-31 Go2 OFF·loopback 조건의 최종 산출물은 다음 경로에 있다.

- `runs/localization/stage13-freeze-domain64-max120/result.json`: stationary bag과
  그 bag으로 만든 저장 지도를 Map Server·AMCL에 연결한 최종 PASS
- `runs/localization/stage13-freeze-domain64-max120/launch.log`: Map Server, AMCL,
  mapping scan과 odometry adapter 실행 로그
- `runs/localization/stage13-freeze-domain64-max120/player.log`: paused rosbag
  readiness·resume·playback 로그
- `runs/localization/stage13-freeze-domain64/result.json`: 기본 participant-index
  탐색 범위에서 실패한 첫 실행의 결과

최종 run은 `/scan` `300`, `/odom` `2923`, finite AMCL pose `1`, active lifecycle,
AMCL 단독 `map → odom`, command·control 0과 clean teardown을 통과했다. 첫 실행은
Map Server·AMCL lifecycle 문제가 아니라 `mapping_cloud_gate`와 rosbag player가
Domain 64 participant index를 얻지 못해 실패했다. 같은 source·map·bag에서
CycloneDDS `ParticipantIndex=auto`, `MaxAutoParticipantIndex=120`만 명시하자 통과했다.
이 조건은 software-only 재현 설정이며 production source 기본값으로 추가하지 않았다.

## Domain 65 Nav2 shadow

2026-08-31 합성 전체 Nav2의 최종 산출물은 다음 경로에 있다.

- `runs/nav2_shadow/stage13-freeze-domain65-max120/summary.json`: 여섯 시나리오
  전체 PASS summary
- 같은 디렉터리의 `success`, `cancel`, `blocked_goal`, `outside_map_goal`,
  `planner_failure`, `no_progress`: 시나리오별 result와 fixture·Nav2 로그

success는 `SUCCEEDED`, cancel은 `CANCELED`, 나머지 네 failure class는 기대한
`ABORTED`였다. 모든 Nav2 lifecycle node가 active였고 fixture가 `/clock`,
`map → odom`, `odom → base`를 단독 소유했다. 물리 command·Go2 control·Unitree
node, 종료 뒤 잔류 process·node·TF owner는 모두 0이었다. 이 결과는 합성
`NavigateToPose` 계약 검증이며 실제 주행 합격이 아니다.

## Stage 13 software-only freeze

동결 범위, runtime checksum, build/test 결과, warning과 deferred 항목은
`verification/software_only_freeze.md`와
`verification/structured/software_only_freeze.yaml`에서 해석한다. 재검증용
SHA-256 목록은 `verification/software_only_freeze.sha256`이며 manifest 자기 자신은
목록에서 제외한다.

## DimOS static TF profile A/B

2026-08-28 같은 external short bag을 기존 `project_default`와 source-aligned
`dimos_replay` TF profile로 각각 1.0배속 재생했다.

- `runs/mapping_tf_ab/20260828_dimos_tf_ab_retry1/summary.json`: 두 profile의 최종
  stream·`map → odom` 연속성·artifact·teardown 비교
- `runs/mapping_tf_ab/20260828_dimos_tf_ab_retry1/project_default/`: 기존 project
  TF 결과·지도·pose graph·로그
- `runs/mapping_tf_ab/20260828_dimos_tf_ab_retry1/dimos_replay/`: DimOS source
  extrinsic 결과·지도·pose graph·로그
- `runs/mapping_tf_ab/20260828_dimos_tf_ab/`: 기존 Domain 63 RViz2로 participant가
  고갈돼 player 시작 전에 중단된 첫 실행 근거

두 최종 run 모두 source cloud `1843/1843`, odometry `18026/18026`, map·pose graph
저장과 clean teardown을 통과했지만 translation·yaw 연속성 기준을 초과했다. 따라서
summary의 `overall`은 `failed`, `toggle_confirmed`는 `false`다. 정확한 profile 값,
threshold, 개선량과 checksum은
`records/experiments/go2_local_navigation_dimos_tf_profile_ab_20260828.yaml`을 따른다.

## DimOS scan projection A/B

TF를 `dimos_replay`로 고정하고 같은 120초 short bag에서 단일 cloud와 10-frame
odometry 보정 누적을 비교했다.

- `runs/mapping_scan_ab/20260828_dimos_scan_ab_round3/summary.json`: 최종 scan 밀도·
  `map → odom` 연속성·artifact·teardown 비교
- `runs/mapping_scan_ab/20260828_dimos_scan_ab_round3/raw_single/`: 단일 validated
  cloud projection 결과
- `runs/mapping_scan_ab/20260828_dimos_scan_ab_round3/dimos_odom_accumulated/`:
  odom frame 10-cloud 누적 결과
- `runs/mapping_scan_ab/20260828_dimos_scan_ab_round1/`: candidate player가 Domain 63
  participant를 얻지 못한 실행
- `runs/mapping_scan_ab/20260828_dimos_scan_ab_round2/`: padded mixed PointCloud2의
  Humble TF 변환 결함을 재현한 실행

최종 run은 두 변형 모두 cloud `1843/1843`, odometry `18026/18026`, map·pose graph
저장·재로딩과 clean teardown을 마쳤다. 누적 profile은 beam 밀도와 기준 초과 횟수를
줄였지만 최대 translation·yaw step 기준은 계속 실패했다. 정확한 수치·checksum과
실패 실행의 취급은
`records/experiments/go2_local_navigation_dimos_scan_projection_ab_20260828.yaml`을
따른다.

## DimOS SLAM continuity replay-only resolution

후속 canonical 120초 bag final A/B와 recorder-free repeat 산출물은 다음 경로에 있다.

- `runs/mapping_slam_fix/20260828_stable_emit3_loop_on_final_ab/summary.json`: baseline
  `raw_single` failed와 `dimos_odom_accumulated_emit3` passed의 최종 비교
- `runs/mapping_slam_fix/20260828_stable_emit3_repeat_1/summary.json` 및
  `runs/mapping_slam_fix/20260828_stable_emit3_repeat_2/summary.json`: 동일 candidate
  maximum과 accounting을 재현한 두 clean repeat
- `runs/mapping_slam_fix/20260828_stable_emit3_full_build_test_round2/build-test.txt`:
  7 packages, 267 tests, 0 errors/failures, 3 skipped

위 build artifact는 `go2_validation` package 분리 전 canonical replay 실행에 속하는
역사적 근거다. 분리 후 현재 source는 AGX의 새 임시 install space에서 8 packages,
287 tests, 0 errors/failures, 3 skipped로 다시 검증했다. 새 검증은 runtime data를
만들지 않았으며 구조·profile 경계와 결과는 로컬 record
`go2-local-navigation-validation-package-refactor-20260828`에서 관리한다.

candidate는 frame/emit `3/3`, input/retry `64/64`, converter `min_height=-0.10`,
queue `64`로 실행했다. 최종 A/B는 response expansion `false`, loop closing `true`를
양쪽 공통값으로 통제하고 candidate에만 coarse `0.1745`를 적용했다. `raw_single`
baseline coarse는 `0.349`였고 global launch default `0.349/true/true`는 유지했다.
`raw_single` baseline은
`0.5535118551684397 m` / `0.3678837777692796 rad`로 failed였고, candidate의
map-correction maximum은 `0.41047936583987504 m` / `0.1800169168112875 rad`였고
exceedance·unaligned·regressive는 모두 `0`이었다. cloud accounting은
`1843/1842/614`, intrinsic drop `1`, overflow/pending/output regression `0/0/0`이며
artifact 저장·재로딩과 clean teardown을 포함한다.

이는 Go2 OFF, physical execution·command publication 없는 정확한 canonical
bag/profile의 replay-verified 근거다. 앞선 TF A/B와 10-frame scan A/B의 failed/open
결과는 이 emit3 delivery와 profile-scoped coarse override 전 조건의 역사적 근거로
보존한다. RViz 상대 시각 관찰은
`/home/tjwocjf0915/research/Go2/.omo/evidence/slam-stable-emit3-rviz-20260828/terminal.txt`에
있으며 지도 정확도 근거가 아니다. 전체 한계는
`records/experiments/go2_local_navigation_dimos_slam_continuity_resolution_20260828.yaml`을
따른다.

최종 profile·상태·한계의 canonical 조회 기준은 record
`go2-local-navigation-dimos-slam-continuity-resolution-20260828`와 같은 stem의 Markdown이다.
위 artifact는 이 정확한 replay 조건의 근거일 뿐 live 적합성·지도 정확도·localization·전체
Nav2의 근거가 아니다.

## DimOS coarse search 7값 sweep

- `runs/mapping_coarse_sweep/20260829_coarse_search_7value_round1/summary.json`:
  canonical short bag·`dimos_replay`·`dimos_odom_accumulated_emit3`를 고정하고
  coarse search `0.0698~0.2792 rad` 7개를 비교한 원문이다.
- 결과는 4 passed·3 failed이며 `0.2094 rad`부터 yaw 연속성 기준을 초과했다.
- `0.0698`은 `0.3752606931950222 m` / `0.08927691681128769 rad`로 이번 범위의
  continuity 우선 후속 후보지만, canonical 채택이나 지도 정확도 근거는 아니다.
- 모든 후보는 cloud `1843/1842/614`, intrinsic drop `1`, command/control `0`,
  map·pose graph 저장·재로딩과 clean teardown을 완료했다.
- summary SHA-256은
  `c6a4213e10b823c5548d23b8517a0d861e51d07c65701b345690d4a5c582b4db`이며,
  canonical 설명·한계는 record
  `go2-local-navigation-dimos-coarse-search-sweep-20260829`를 따른다.

## DimOS coarse search lower-band 0~4도

- `runs/mapping_coarse_sweep/20260829_lower_search_0to4deg_round1/summary.json`:
  같은 canonical short bag·TF·scan·SLAM 조건에서 `0~4°` 5개 후보를 비교한
  원문이다.
- 5개 후보가 모두 passed했다. `0°`는 최대 yaw
  `0.01852879921693318 rad`, `1°`는 최대 translation
  `0.2800537845414384 m`로 각각 해당 지표의 최저값이었다.
- cloud `1843/1842/614`, intrinsic drop `1`, overflow/pending/regression
  `0/0/0`, artifact 저장·재로딩과 clean teardown은 모든 후보에서 통과했다.
- summary SHA-256은
  `158fedab211baf51142152ea08862a996a36558a06db295689bc634ecb83b117`이며,
  상세 판정·한계는 record
  `go2-local-navigation-dimos-lower-search-20260829`를 따른다.
- `0°`와 `1°`는 각각 yaw·translation 우선 replay 후보로만 기록하고,
  기존 canonical `0.1745 rad`는 변경하지 않았다.

## Map PNG export

관리 대상인 `data/`·`src/` 범위의 occupancy PGM 68개를 원본 옆에 동일 stem의
PNG sidecar로 내보냈다. 예를 들어 `occupancy.pgm`은 `occupancy.png`가 되며,
원본 PGM과 `occupancy.yaml`은 그대로 유지한다. `build/`·`install/` 아래의
생성 복사본은 소스 산출물이 아니므로 별도 PNG를 관리하지 않는다. `data/runs/`
아래의 PNG도 실행 산출물과 동일하게 Git 관리 대상이 아니다.

각도에 따른 형상 변화를 비교할 수 있도록 다음 폴더에 2026-08-29 coarse sweep
결과 12개를 별도로 모았다.

- `.user/img/slam_map/angle_sweep_20260829/`: lower-band `0~4°` 5개와 기존
  coarse `4~16°` 7개 PNG
- `../src/go2_nav2/maps/shadow_blocked.png`, `shadow_open.png`: 합성 map의 PNG
  sidecar

비교 폴더의 PNG는 시각 확인용이다. 각 map의 resolution·origin·threshold와
실행 출처는 원래 실행 디렉터리의 `occupancy.yaml`과 폴더 README를 기준으로 하며,
PNG만으로 지도 정확도나 canonical angle 채택을 판단하지 않는다.
