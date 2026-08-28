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
