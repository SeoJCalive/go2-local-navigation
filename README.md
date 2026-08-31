# go2_local_navigation

## 목적

Unitree Go2와 Jetson AGX Orin에서 센서 계약을 읽기 전용으로 먼저 검증하고,
이후 장애물 정보 보고, mapping, Nav2 costmap, 목적지 주행으로 확장하기 위한
Python 기반 ROS 2 프로젝트다.

현재 상태는 `candidate`(후보)다. static TF와 LiDAR 입력 형식·QoS·주기는 AGX에서
검증했으며, read-only LiDAR acceptance와 odometry source probe도 AGX에서
검증했다. odometry source는 `header=odom`, `child=base_link`로 관찰됐고,
프로젝트는 `base_link → base` mapping과 `go2_state_estimation/odometry_adapter_node`
owner를 수용했다. adapter runtime에서 `/odom`과 `odom → base` dynamic TF를
확인했지만, 이 mapping은 공식 onboard frame 동일성이나 물리 calibration으로
승격하지 않는다. 2026-08-26에는 정지 상태 raw bag record/replay와
`/perception/obstacle_candidates` output을 검증했다. 2026-08-27에는 motion adapter
비동작 dry-run과 Nav2 local costmap·controller preview를 검증했으며, 실제 Sport
control publisher는 생성하지 않았다. 같은 날 30초 통합 비동작 preflight에서 필수
TF·sensor·odom·obstacle candidate·local costmap, 안전 gate, AGX 자원과 clean
teardown을 함께 확인했고 46개 check가 모두 PASS였다. 이어서 30초 smoke는 47개
check가 모두 PASS했고 30분 soak는 46 PASS·1 WARN이었다. 유일한 경고는 정지 yaw
누적 drift `0.248012 rad`이며, 최종 장착 전 해결된 사실로 승격하지 않는다.

같은 날 software-only Wave 1·2에서 Domain 61 합성 fault 10개와 Domain 62 정지·외부
replay 입력을 검증했다. 외부 자료는 cloud `17,776`, odometry `173,616`개를 가진
full canonical bag과 120초 short bag으로 변환했으며, 두 ingress 모두 `/scan` frame
`base`, command publisher 0과 clean teardown을 통과했다. 이 결과는 mapping 입력과
fault recovery 근거이며 SLAM 지도 품질, localization 또는 목적지 도달 근거가 아니다.

이 자료의 `external`은 외장 센서가 아니라 외부 출처 dataset이라는 뜻이다. raw
`go2_china_office_indoor.mcap`은 8개 channel을 가지며 derived canonical bag은
`/utlidar/cloud`와 `/utlidar/robot_odom` 두 topic만 사용한다. topic·frame·rate와 pinned
DimOS 코드에 따라 센서 계열은 Go2 built-in L1 ULIDAR로 강하게 추론하지만 hardware
manifest가 없어 `unverified`다. 같은 source 디렉터리의 Mid-360·Point-LIO DB는
canonical short·full 계보에 포함되지 않는다.

이어 Domain 63 loopback에서 stationary와 external-full을 1.0배속으로 재생해 두
SLAM mapping run을 완료했다. `/map`, SLAM Toolbox 단독 `map → odom`, occupancy
map·pose graph 저장과 재로딩, command publisher 0과 잔류 process 0을 확인했다.
이는 map 생성·저장 경계의 replay 근거이며 지도 정확도·loop closure 품질이나
localization·NavigateToPose의 근거로 승격하지 않는다.

2026-08-31에는 Go2를 끈 채 Domain 64 loopback에서 stationary bag과 그 bag으로
저장한 지도를 다시 연결했다. Map Server와 AMCL은 active였고 `/scan` `300`개,
`/odom` `2923`개, finite AMCL pose `1`개와 AMCL 단독 `map → odom` owner를 확인했다.
command·control publisher와 종료 뒤 잔류 node·process는 모두 0이었다. 이 결과는
saved-map localization 연결성과 소유권의 replay 검증이며 위치 정확도 증명이 아니다.

같은 날 Domain 65에서는 합성 `/clock`, `map → odom`, `odom → base`, odometry만
제공하고 Nav2 전체 stack을 `/go2_nav2/shadow_cmd_vel`에 격리했다. success는
`SUCCEEDED`, cancel은 `CANCELED`, blocked goal·outside-map goal·planner failure·
no-progress는 각각 기대한 `ABORTED`로 종료됐다. 여섯 시나리오 모두 lifecycle,
costmap, TF owner, 물리 command publisher 0과 clean teardown을 통과했다. 이는 합성
입력에서의 Nav2 action 계약 근거이며 실제 Go2 목적지 도달이나 장애물 회피 근거가
아니다.

2026-08-28에는 DimOS 저장소의 Go2 LiDAR extrinsic을 외부 replay 전용
`dimos_replay` TF profile로 추가하고 같은 120초 short bag을 기존
`project_default`와 A/B했다. 최대 translation step은 `10.014682 → 8.768064 m`,
yaw step은 `0.545220 → 0.366329 rad`로 감소했지만 두 profile 모두 현재 연속성
기준 `0.5 m`·`0.2 rad`를 초과했다. 따라서 source extrinsic은 유효한 보정이지만
순간이동의 단독 원인으로 확정하지 않으며, 기존 실물 기본 TF도 변경하지 않는다.

같은 날 TF를 `dimos_replay`로 고정하고 raw 단일 cloud와 10-frame odometry 보정
누적을 A/B했다. 누적 profile은 유효 beam 중앙값을 `36 → 251`, translation·yaw
기준 초과 횟수를 각각 `487 → 94`, `49 → 4`로 줄였지만 최대 step은 여전히
`8.221949 m`·`0.358414 rad`였다. 따라서 scan 희소성은 부분 원인으로 남기고,
누적 profile은 외부 replay 실험 후보로만 유지한다.

이후 canonical 120초 DimOS bag만을 다시 재생한 최종 resolution에서
`dimos_odom_accumulated_emit3`는 frame/emit `3/3`, input/retry `64/64`, converter
`min_height=-0.10`, queue `64`와 profile-scoped coarse `0.1745`를 사용했다.
최종 A/B는 response expansion `false`, loop closing `true`를 양쪽 공통값으로
통제했고 candidate에만 coarse `0.1745`를 적용했다. baseline coarse는 `0.349`였으며
전역 launch 기본값 `0.349/true/true`는 바꾸지 않았다. baseline `raw_single`은
`0.5535118551684397 m` / `0.3678837777692796 rad`로 failed였지만 candidate는
`0.41047936583987504 m` / `0.1800169168112875 rad`, exceedance·unaligned·regressive
모두 `0`으로 passed였다. cloud `1843` 수신·`1842` 처리·`614` output, intrinsic
drop `1`, overflow/pending/regression `0`과 occupancy·pose graph 저장·재로딩, clean
teardown도 final A/B와 두 repeat에서 동일하게 통과했다. 이는 Go2 OFF,
`physical_execution=false`, `command_publication=false`인 이 정확한 bag/profile의
software replay continuity 결과만 뜻하며, 실물 장착·live fit·지도 ground truth·
localization/Nav2는 계속 미검증이다.

2026-08-29에는 같은 bag·emit3·TF·SLAM 공통값을 유지하고 coarse search만
`0.0698~0.2792 rad` 7개로 바꿔 순차 비교했다. 앞의 네 값은 passed, 뒤의 세 값은
yaw 기준으로 failed였고 `0.0698`은 `0.3752606931950222 m` /
`0.08927691681128769 rad`를 기록했다. 이는 이번 범위의 continuity 우선 후속
후보일 뿐, 최저 시험 경계이며 지도 ground truth·live 조건이 없으므로 기존
replay canonical `0.1745`를 교체하지 않는다.

이어서 같은 조건에서 `0~4°`를 `1°` 간격으로 추가 시험했다. 5개 후보가 모두
passed했고, `0°`는 yaw 최대 `0.01852879921693318 rad`, `1°`는 translation
최대 `0.2800537845414384 m`로 각각 해당 지표의 최저값이었다. 두 지표의 최소값이
같은 후보에 모이지 않았으므로 `0°`·`1°` 모두 replay 후보로만 보존하고 운영
canonical은 `0.1745`로 유지한다. 상세 결과와 한계는
`records/experiments/go2_local_navigation_dimos_lower_search_20260829.md`와
같은 stem의 YAML projection에서 조회한다.

후속 causal A/B에서는 동일 입력에서 `use_scan_matching`만 끈 경우 short·full corrected
pose가 odometric pose와 같아지고 held-out 재현성이 회복됐다. 따라서 최초 pose 열화는
Karto sequential scan matching으로 확인했다. 반면 full loop-off는 장거리 edge를
제거해도 회복되지 않아 explicit loop closure는 주원인에서 제외했다. matching-off에도
PGM fan·streak와 occupancy support 손실이 남으므로 occupancy 생성은 `open`이다.
matching-off는 진단 음성 대조군이며 package/default/canonical 설정은 변경하지 않았다.

2026-08-30에는 matching-off의 동일 admitted scan·pose를 고정해 occupancy만 분석했다.
`unsupported_occupied`는 첫 측정 prefix인 node `30` 또는 그 이전부터 지속됐다.
`multi_segment_occupied` 후보는 primary skeleton을 `4000 → 4`셀로 줄여 수치 gate를
통과했지만, blind visual QA에서는 baseline보다 fan/streak와 topology 가독성이
악화됐다. 따라서 최종 winner는 없고 full candidate 실행은 `0`회다. 상태는
`root-cause-classified`이며 `src/`, TF, scan profile, SLAM/default는 변경하지 않았다.

최종 profile·상태·한계는 canonical record
`go2-local-navigation-dimos-slam-continuity-resolution-20260828`의
`records/experiments/go2_local_navigation_dimos_slam_continuity_resolution_20260828.md`와
YAML projection에서 조회한다. final A/B·repeat·build/test run artifact 경로는
[`data/README.md`](data/README.md)에 보존한다.

## 작업 위치와 규칙

이 프로젝트는 저장소의
`remote_agx_home/go2_projects/projects/go2_local_navigation` 경로로 AGX SSHFS
mount되어 있다. 저장소 루트 `AGENTS.md`가 이 경로에 재귀 적용되므로 프로젝트
전용 `AGENTS.md`는 두지 않는다.

프로젝트의 안전 경계와 패키지별 책임은 이 README와 각 패키지 README에서
관리한다. 확인되지 않은 센서 field·QoS·TF·extrinsic·odometry는 계속
`unverified` 또는 `TBD`로 기록한다.

## 현재 기초 패키지

- `go2_sensor`: 센서 topic 계약과 acceptance 경계
- `go2_state_estimation`: odometry source 관찰과 `odom → base` adapter
- `go2_perception`: LiDAR obstacle candidate 보고 경계
- `bringup`: launch·설정과 재사용 가능한 통합 preflight observer
- `description`: 공식 Go2 전체 URDF와 TF 계약
- `go2_control`: 비동작 기본 motion 계약, 제한·watchdog·Sport request 변환 후보
- `go2_nav2`: 비동작 Nav2 preview, experimental SLAM·saved-map localization과 합성 전체 Nav2 runtime asset
- `go2_validation`: fault·replay·mapping·localization·Nav2 shadow·통합 preflight의 software-only validation orchestration

현재 범위에는 프로젝트 내부 `/go2_control/cmd_vel_candidate`와
`/go2_control/sport_request_preview`가 포함된다. 실제 `/api/sport/request` publish,
`/lowcmd`, stand/walk와 navigation motion 실행은 포함하지 않는다.

## 검증 진입점

모듈별 현재 검증 수준, 근거 record, 남은 합격 조건과 재검증 시점은
[`verification/README.md`](verification/README.md)에서 찾는다. 검색·비교용 중앙
인덱스는 `verification/structured/acceptance_matrix.yaml`, 프로젝트 범위 manifest는
`verification/structured/project_manifest.yaml`이며, 실제 수치와 실행 상세는 기존
package contract와 로컬 `records/`가 계속 소유한다.

9단계 통합 검증은 다음 명령으로 반복한다.

```bash
source /home/bi-agx1/go2_runtime/go2_agx_ros2_humble_env.sh
cd /home/bi-agx1/go2_projects/projects/go2_local_navigation
source install/setup.bash
ros2 run go2_validation integrated_preflight \
  --ros-args -p duration_sec:=30 -p run_label:=stage9
```

9단계 성공 run은 `data/runs/preflight/20260827_020502_stage9/result.json`이다.
10단계는 `20260827_150959_stage10_smoke`와
`20260827_151053_stage10_soak`이며, 실행은 완료됐지만 soak의 yaw drift 경고를
보존한다. 정확한 수치와 checksum은 로컬
`records/experiments/go2_local_navigation_stationary_soak_20260827.md`와 같은
stem의 YAML projection을 따른다. runtime JSON과 로그는 Git 대상이 아니다.

`go2_control`은 현재 AGX graph와 공식 Unitree Request schema를 근거로 구현됐고
motion adapter와 read-only trial recorder를 포함한 22개 package 테스트를 통과했다.
recorder는 실제 `/odom` 5947개를 기록하고 control publisher 없이 clean exit했다.
기본 이중 gate는 닫혀 있으며 실제 command 전송은 수행하지 않았다. 2026-08-28
`go2_validation` 분리와 profile 경계 반영 뒤 AGX 실제 workspace를 clean build해
표준 `install/`에 반영했다. 전체 8 packages, 287 tests, 0 failures, 0 errors,
3 skipped와 설치 executable `go2_validation=10`, `go2_nav2=0`을 확인했다. 원시
로그·JUnit archive·install surface·직접 QA와 checksum은
`.omo/evidence/validation-package-refactor-closeout-20260828/README.md`에 보존한다.
이 수치는 software-only 구조·계약 검증이며 replay나 live 성능의 추가 합격
근거가 아니다. 2026-08-29 sweep runner 추가 뒤에는 AGX에서 `go2_validation`을
재빌드하고 157개를 수집해 154 passed·3 skipped·0 failures·0 errors와 현재 설치
executable `go2_validation=11`을 확인했다. 2026-08-31 saved-map localization과
Nav2 shadow를 추가한 Stage 13 동결 검증은 전체 8 packages, 318 tests, 0 failures,
0 errors, 5 skipped와 설치 executable `go2_validation=14`, `go2_nav2=0`을 확인했다.
Domain 64·65의 조건·checksum과 주장 한계는
[`software-only freeze`](verification/software_only_freeze.md)를 따른다.

## 로봇 전원 없이 RViz2로 URDF·TF 보기

Go2가 꺼져 있거나 AGX와 Go2 Ethernet이 연결되지 않은 상태에서는
`bringup/go2_offline_rviz.launch.py`로 공식 URDF와 프로젝트 static TF를 RViz2에서
확인할 수 있다. 이 launch는 CycloneDDS를 loopback으로 제한하므로 Go2 통신이
필요하지 않다.

### 최초 1회 환경 설치

AGX에서 다음 패키지를 한 번 설치한다.

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-rviz2 ros-humble-joint-state-publisher
```

### 프로젝트 build

프로젝트 파일이나 launch 설정을 변경한 뒤 AGX에서 build한다.

```bash
source /home/bi-agx1/go2_runtime/go2_agx_ros2_humble_env.sh
cd /home/bi-agx1/go2_projects/projects/go2_local_navigation
colcon build --symlink-install --packages-select description bringup
source install/setup.bash
```

### RViz2 실행

AGX의 그래픽 로그인 세션에 있는 터미널에서 다음을 실행한다.

```bash
source /home/bi-agx1/go2_runtime/go2_agx_ros2_humble_env.sh
cd /home/bi-agx1/go2_projects/projects/go2_local_navigation
source install/setup.bash
ros2 launch bringup go2_offline_rviz.launch.py
```

RViz2에는 다음이 자동으로 표시된다.

- Fixed Frame: `base`
- 공식 Go2 `RobotModel`
- `TF` frame 축과 이름
- Grid
- `base → imu`, `base → radar`, `base → front_camera`,
  `base → utlidar_lidar` 연결

이 launch가 시작하는 `joint_state_publisher`는 URDF에서 기본 joint state를
합성하는 시각화용 publisher다. 실제 관절 상태가 아니며 `/utlidar/*`,
`/utlidar/robot_odom`, `/odom`, `/cmd_vel`, Unitree service, motor command를
사용하지 않는다. 따라서 이 상태에서는 실시간 LiDAR·IMU·odom·map은 표시되지
않는다.

종료할 때는 RViz2가 실행 중인 터미널에서 `Ctrl+C`를 누른다. 이 offline launch는
static TF와 합성 joint state를 내부에서 함께 시작하므로
`go2_static_tf.launch.py`나 `go2_odometry_adapter.launch.py`를 별도로 실행하지
않는다.

SSH 터미널에서 실행해 `Unable to open display`가 나오면 오류가 아니라 GUI display가
없는 상태다. AGX의 실제 그래픽 화면 터미널에서 실행하거나 X/Wayland display가
전달된 세션을 사용해야 한다.

## TF 설계

```text
map → odom → base
                ├── imu
                ├── radar
                └── utlidar_lidar
```

`map→odom`은 SLAM/localization, `odom→base`는
`go2_state_estimation/odometry_adapter_node`,
센서 frame 연결은 실제 값이 확인된 뒤 해당 구성요소가 소유한다. 현재
canonical model과 프로젝트 root는 모두 `base`이며, 공식 native sensor 의미는
`imu`=차체 IMU, `radar`=기본 Unitree LiDAR, `front_camera`=기본 내장
카메라로 기록한다. runtime `/utlidar/cloud`의 frame_id는
`utlidar_lidar`로 확인했고 static TF 연결도 runtime에서 검증했다. URDF 기준값은
`description/urdf/go2_description.urdf`에서 확인하고, 좌표 확인 이미지는
`description/urdf/go2_description_coordinate_check.png`에서 확인한다.

static sensor TF는 `bringup/config/static_tf_profiles.yaml`에서 profile별로
관리한다. 기본 `project_default`는 기존 실물·일반 launch에 계속 사용하고,
`dimos_replay`는 해당 외부 bag을 재생할 때만 명시한다. source frame 대응은 외부
replay용 project mapping이며 실물 센서의 물리 calibration으로 승격하지 않는다.

하나의 TF edge에는 하나의 publisher만 둔다. 센서 입력과 perception 보고는
읽기 전용으로 유지하며, `/cmd_vel`, Unitree control service, motor command,
stand/walk, navigation command는 별도 승인 전까지 추가하거나 실행하지 않는다.

공식 URDF는 구조·기준값을 보관하는 asset이지 실행 승인이나 현재 runtime
TF 검증 결과가 아니다. `map→odom`과 `odom→base`는 URDF fixed
joint로 추가하지 않는다.

## 다음 확인 단계

1. 10단계 정지 smoke·soak는 실행 완료 상태로 유지하되 yaw drift 경고를 후속 시험에 연결한다.
2. 11단계 software fault recovery와 12단계의 mapping·saved-map localization·합성 Nav2 shadow는 software-only 범위에서 완료했다.
3. 외부 replay continuity와 occupancy 품질은 별도 판정으로 유지한다. canonical continuity는 정확한 bag/profile에서만 통과했고 occupancy production 후보는 없으며, Domain 64 localization 연결성과 Domain 65 Nav2 action 결과를 지도·위치 정확도로 승격하지 않는다.
4. 13단계 software-only freeze 당시 보류됐던 Domain 0 중 live scan·정지 SLAM mapping은 12-L supplement에서 `stationary-onboard-verified`로 보완했다. live localization·no-goal Nav2 observer와 지도 정확도는 계속 보류한다.
5. 다음 실행 단계인 14단계에서 AGX 최종 고정과 footprint·케이블·전원·열·센서 시야를 기록한다.
6. 15단계는 이동 가능한 전원, 단일 command owner와 최신 승인이 있을 때만 축·StopMove를 제한적으로 검증한다.

센서·odometry source probe 결과는
`records/experiments/go2_local_navigation_sensor_odometry_probe_20260823.md`와
같은 stem의 YAML projection에 보존한다.

adapter mapping 결정과 runtime 결과는 각각
`records/decisions/go2_local_navigation_odom_frame_mapping_20260823.md`,
`records/experiments/go2_local_navigation_odom_adapter_20260823.md`와 같은 stem의
YAML projection에 보존한다.

정지 상태 bag/replay와 perception 결과는
`records/experiments/go2_local_navigation_stationary_bag_perception_20260826.md`와
같은 stem의 YAML projection에 보존한다.

motion adapter dry-run과 Nav2 preview 결과는 각각
`records/experiments/go2_local_navigation_motion_adapter_dry_run_20260827.md`,
`records/experiments/go2_local_navigation_nav2_non_actuating_preview_20260827.md`와
같은 stem의 YAML projection에 보존한다.

통합 비동작 preflight 결과는
`records/experiments/go2_local_navigation_integrated_preflight_20260827.md`와 같은
stem의 YAML projection에 보존한다.

정지 smoke·soak와 trial recorder QA 결과는 각각
`records/experiments/go2_local_navigation_stationary_soak_20260827.md`,
`records/experiments/go2_local_navigation_trial_recorder_readonly_qa_20260827.md`와
같은 stem의 YAML projection에 보존한다.

stationary·external-full SLAM mapping 결과는
`records/experiments/go2_local_navigation_slam_mapping_replay_20260827.md`와 같은
stem의 YAML projection에 보존한다.

Domain 0 live scan·정지 SLAM mapping supplement 결과는
`records/experiments/go2_local_navigation_domain0_live_mapping_shadow_20260831.md`와
같은 stem의 YAML projection에 보존한다.

DimOS source extrinsic과 120초 TF profile A/B 결과는
`records/experiments/go2_local_navigation_dimos_tf_profile_ab_20260828.md`와 같은
stem의 YAML projection에 보존한다.

DimOS 단일 cloud와 10-frame odometry 보정 누적 A/B 결과는
`records/experiments/go2_local_navigation_dimos_scan_projection_ab_20260828.md`와
같은 stem의 YAML projection에 보존한다.

canonical emit3 replay-only resolution은
`records/experiments/go2_local_navigation_dimos_slam_continuity_resolution_20260828.md`와
같은 stem의 YAML projection에 보존한다. 앞선 TF/10-frame A/B의 failed/open 결과는
그 최종 profile 적용 전 조건의 역사적 근거로 보존한다.

최신 pose·occupancy 원인 분리는
`records/experiments/go2_local_navigation_dimos_slam_causal_attribution_20260829.md`와
같은 stem의 YAML projection에 보존한다. 이 후속 기록은 continuity pass를 폐기하지
않고 당시 근본 원인 설명만 부분 대체한다.

고정 pose occupancy breakpoint와 후보 판정은
`records/experiments/go2_local_navigation_dimos_occupancy_quality_resolution_20260830.md`와
같은 stem의 YAML projection에 보존한다. 수치 gate의 winner와 blind visual-approved
winner를 분리하며, 이 기록의 후보는 live/default 설정으로 승격하지 않는다.
