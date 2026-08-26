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
teardown을 함께 확인했고 46개 check가 모두 PASS였다.

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
- `go2_nav2`: 비동작 Nav2 local costmap·controller preview와 통합 runner

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
ros2 run go2_nav2 integrated_preflight \
  --ros-args -p duration_sec:=30 -p run_label:=stage9
```

최신 성공 run은 `data/runs/preflight/20260827_020502_stage9/result.json`이며,
상세 근거는 로컬
`records/experiments/go2_local_navigation_integrated_preflight_20260827.md`와 같은
stem의 YAML projection에 보존한다. runtime JSON과 로그는 Git 대상이 아니다.

`go2_control`은 현재 AGX graph와 공식 Unitree Request schema를 근거로 구현됐고
AGX runtime dry-run과 14개 자동 테스트를 통과했다. 기본 이중 gate는 닫혀 있으며
실제 command 전송은 수행하지 않았다. Nav2 controller preview는 내부 candidate와
Sport preview까지만 연결됐고, 정지 상태에서 local costmap `1.667 Hz`와 clean
teardown을 확인했다.

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

하나의 TF edge에는 하나의 publisher만 둔다. 센서 입력과 perception 보고는
읽기 전용으로 유지하며, `/cmd_vel`, Unitree control service, motor command,
stand/walk, navigation command는 별도 승인 전까지 추가하거나 실행하지 않는다.

공식 URDF는 구조·기준값을 보관하는 asset이지 실행 승인이나 현재 runtime
TF 검증 결과가 아니다. `map→odom`과 `odom→base`는 URDF fixed
joint로 추가하지 않는다.

## 다음 확인 단계

1. 두 motion gate를 닫은 채 10단계 정지 smoke·soak를 수행한다.
2. AGX를 최종 고정한 뒤 footprint, 케이블, 열, 센서 위치를 다시 확인한다.
3. 동적 활동이 가능한 조건에서 mapping과 `map → odom`을 검증한다.
4. 실제 output은 별도 승인 후 축 방향·제동·StopMove부터 제한적으로 검증한다.

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
