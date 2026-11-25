# go2_local_navigation

Unitree Go2의 MuJoCo 상태에서 LiDAR·odometry를 만들고, SLAM mapping과 저장 지도
AMCL/Nav2를 실행하는 ROS 2 Humble workspace다. simulator·RL controller·joystick
FIFO bridge는 별도 process다. 현재 상태는 simulation `candidate`이며 실물 실행
기준으로 사용하지 않는다.

## 구성과 문서

| 경로 | 패키지와 책임 |
| --- | --- |
| `src/description` | canonical URDF·static TF·DDS 환경 판정 |
| `src/go2_inputs` | cloud gate·공식 scan converter 조합·odom adapter |
| `src/go2_nav2` | SLAM·AMCL·Nav2 설정·session guard·tf2 loader |
| `src/go2_mujoco_lidar` | MuJoCo 센서·공개 sim launch·지도 저장 |
| `optional/real_robot` | sensor/state-estimation/perception/control/bringup 5개 실물 후보 |
| `tools/go2_validation` | fault·shadow·preflight 등 개발 검증 패키지 |
| `third_party` | simulator/SDK/MuJoCo와 tf2의 고정 버전 설치 절차 |
| `verification` | 추적 checkpoint와 과거 검증 인덱스 |
| `docs/stabilization-2026-11` | 안정화 변경 이력과 검증 결과 |

- 운영 상세: [SIM_SESSION](src/go2_nav2/SIM_SESSION.md).
- 반복 수용 시험: [acceptance](src/go2_mujoco_lidar/test/acceptance/README.md).
- 무중력·정지 입력 진단: [headless](src/go2_mujoco_lidar/test/headless/README.md).
- 변경 이력: [안정화 색인](docs/stabilization-2026-11/README.md).
- 과거 증거: [verification](verification/README.md), [보관 위치](verification/archive-index.md).

## 최초 준비와 빌드

Ubuntu 22.04·ROS Humble·Unitree ROS 메시지 overlay가 필요하다.
시스템 의존성과 새 환경의 설치 계약은
[simulator bootstrap](third_party/unitree_mujoco/README.md)과
[tf2 bootstrap](third_party/tf2/README.md)을 따른다.
아래 명령은 AGX 프로젝트 루트에서 실행한다. 기존 공유 checkout 대신 전용
dependency root와 프로젝트 기본 build/install/log를 사용한다.

```bash
cd /home/bi-agx1/go2_projects/projects/go2_local_navigation
# ROS overlay를 source하지 않은 clean shell
export SIM_DEPS="$HOME/.local/share/go2-local-navigation-sim"
bash third_party/unitree_mujoco/setup.sh "$SIM_DEPS"
export UNITREE_MUJOCO_ROOT="$SIM_DEPS/unitree_mujoco"
export MUJOCO_ROOT="$SIM_DEPS/mujoco-3.3.6"

source /opt/ros/humble/setup.bash
source /home/bi-agx1/unitree_ros2/cyclonedds_ws/install/setup.bash
bash third_party/tf2/setup.sh
df -h .
colcon build --base-paths src \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+
source install/setup.bash
```

core는 4개다. 전체 10개를 빌드할 때는 `--base-paths src optional/real_robot
tools/go2_validation`을 사용한다. 일반 colcon 탐색은 optional도 발견한다.
`python3 tools/check_package_dependencies.py`로 직접 의존성과 순환을 검사한다.
simulator에는 Sport stamp·IMU 필드와 GUI 정수 epoch clock 패치가 필수이며
설치 스크립트가 이를 적용한다.
bridge는 형제 저장소의 `cleanup/domain-guard`에 있는 nonzero-domain guard와
waypoint 하한 변경을 사용하고 별도 prefix에 빌드한다.

## 시작 순서

한 실행에 사용하지 않는 nonzero domain, joystick 장치/FIFO와 world를 선택한다.
아래 Domain 41은 전용 예다. 기존 Domain 1 stack을 중지하거나 재사용하지 않는다.
수동 Xbox는 현재 `/dev/input/js0`의 장치/FD 동일성을 먼저 확인하며,
FIFO는 자신이 시작할 simulator의 설정과 같은 경로로 만든다.
공유 simulator config를 수정하지 않는다.

터미널 1은 ROS 환경을 source하지 않은 clean shell에서 시작한다.
`unitree_mujoco`는 `../config.yaml`을 읽으므로 전용 checkout의 `simulate/config.yaml`에
이 세션의 joystick 설정이 준비되어 있어야 한다.

```bash
export SIM_DEPS="$HOME/.local/share/go2-local-navigation-sim"
export WORLD=/home/bi-agx1/go2_projects/projects/go2_nav2_mujoco_bridge/worlds/go2_complex_indoor.xml
cd "$SIM_DEPS/unitree_mujoco/simulate/build"
./unitree_mujoco -r go2 -s "$WORLD" -i 41 -n lo
```

터미널 2도 clean shell에서 시작한다.

```bash
cd /home/bi-agx1/go2_projects/projects/go2_rl_locomotion_baseline/deploy/robots/go2/build
./go2_ctrl -n lo -d 41
```

`Connected to robot`·`FSM: Start Passive`를 확인한 뒤 LT+A, 약 3초 기립,
Start+D-pad Down으로 FixStand → Velocity_Down을 선택한다.
터미널 3부터는 다음 공통 ROS 환경을 사용한다.
`SIM_PID`에는 터미널 1에서 시작한 실제 PID를 넣는다.

```bash
source /opt/ros/humble/setup.bash
source /home/bi-agx1/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/bi-agx1/go2_projects/projects/go2_local_navigation/install/setup.bash
export ROS_DOMAIN_ID=41 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 실제 XML의 lo-only interface와 loopback peer로 제한한다.
export CYCLONEDDS_URI="file:///home/bi-agx1/go2_projects/projects/go2_local_navigation/src/go2_mujoco_lidar/config/cyclonedds_loopback.xml"
ros2 launch go2_mujoco_lidar go2_sim_navigation.launch.py \
  simulator_pid:="$SIM_PID" mode:=mapping
```

이 명령이 LiDAR·scan·static TF·odom adapter·SLAM을 한 번씩 시작한다.
센서만 필요하면 `go2_sim_bringup.launch.py simulator_pid:="$SIM_PID"
mapping:=false`를 사용한다. mapping을 포함한 정규 운영은 tf2 pin을 적용하는
`go2_sim_navigation.launch.py`를 사용한다.
world는 simulator의 `-s`에서 읽어 raycast에 전달하고 경로·SHA-256을 기록한다.
기본 센서는 `720×8`, 수직 `±15°`이며 base 방위각별 최소 절대 elevation ray를
고르는 planar-selected cloud가 `/scan` 입력이다. sensor-frame 단일 수평 layer와
다르므로 기본 실행에서 raster override를 생략한다.

터미널 4는 같은 ROS 환경에서 bridge install을 추가 source한다.
`OWN_FIFO`는 터미널 1의 전용 joystick FIFO다. 수동 Xbox 주행에서는 이 bridge를
실행하지 않는다.

```bash
source /home/bi-agx1/go2_projects/projects/go2_nav2_mujoco_bridge/install/local_setup.bash
ros2 run go2_nav2_mujoco_bridge twist_to_joystick --ros-args \
  -p fifo_path:="$OWN_FIFO"
```

bridge build와 FIFO 입력 절차는 형제 저장소의 README를 따른다.
bridge의 legacy Nav2 launch를 동시에 시작하지 않는다.
별도 RViz는 같은 ROS 환경에서 실행하고 tf2가 필요하면
프로젝트 루트에서
`LD_LIBRARY_PATH="$PWD/third_party/tf2/install/tf2/lib:$LD_LIBRARY_PATH" rviz2 -d "$PWD/src/go2_mujoco_lidar/config/simulation.rviz" --ros-args -p use_sim_time:=true`로 pin과 simulation 설정을 적용한다.
Nav2/SLAM native child의 pin은 세션 launch가 강제하고 `TF2_LOADED`로 확인한다.

## 지도 저장과 재로딩

주행을 멈추고 mapping이 살아 있는 동안 같은 ROS 환경의 별도 셸에서 저장한다.
output은 suffix 없는 새 절대 경로다.

```bash
ros2 run go2_mujoco_lidar sim_save_map \
  --simulator-pid "$SIM_PID" --output "$PWD/runs/gui-map/route"
```

YAML·PGM과 world/domain/hash를 담은 `.session.json`을 저장한다.
posegraph/data와 원자적 SLAM checkpoint 저장은 별도다.
mapping ROS launch를 종료하고 모든 owned ROS PID가 사라진 뒤 simulator/RL은
유지한 채 다음을 실행한다.

```bash
ros2 launch go2_mujoco_lidar go2_sim_navigation.launch.py \
  simulator_pid:="$SIM_PID" mode:=localization \
  map:="$PWD/runs/gui-map/route.yaml" initial_pose:=0.0,0.0,0.0
```

initial_pose는 저장 지도에서 현재 base의 실제 x,y,yaw다. 원점은 형식 예다.
AMCL이 새 pose를 수락한 뒤에만 Nav2 lifecycle이 시작된다.
`map → odom`은 mapping에서 SLAM, localization에서 AMCL 하나만 소유한다.
동일 domain 세션은 OS lock으로 직렬화한다.
localization 종료는 [SIM_SESSION 종료 순서](src/go2_nav2/SIM_SESSION.md#종료-순서)의
lifecycle FINALIZED 확인 후 ROS launch → bridge → RL → simulator 순서로 진행한다.
physics reset은 전체 세션 재시작 조건이다.

## 검증과 격리

```bash
# 위 build prefix와 같은 전체/핵심 package 범위를 사용한다.
colcon test --base-paths src \
  --executor sequential --return-code-on-test-failure --ctest-args -j1
colcon test-result --verbose
```

실제 simulator→RL→SLAM→저장→AMCL/Nav2→FIFO 경로는
[추적 acceptance](src/go2_mujoco_lidar/test/acceptance/README.md)를 사용한다.
기본 13점 mapping, 다섯 goal, 독립 qpos 오차, 비바닥 접촉, 입력 reset, lifecycle
FINALIZED와 owned 잔류 0을 판정하며 일반 colcon test에는 등록하지 않는다.

- Domain 0은 정규 sim/bridge 경로에서 거부한다. guard 우회 옵션은 운영에 쓰지 않는다.
- 모든 physics/RL/ROS process는 같은 domain과 loopback 격리를 사용한다.
- build/install/log와 tf2 prefix는 프로젝트의 Git 제외 영구 경로를 사용한다.
- 소유자 GUI·Xbox·RViz 점검은 [SIM_SESSION GUI 절차](src/go2_nav2/SIM_SESSION.md#소유자-gui-시험)를 따른다.
- 다른 실행의 PID에 signal을 보내지 않는다.
- 소유한 PID/start tick/session과 FIFO만 정리하고 기존 Domain 1 identity를 보존한다.
- 자동 테스트 통과는 구현 계약의 일치다. 실제 지도·위치 정확도와 실물 안전성은 별도다.

## 한계와 미확인

- F02의 동일 source 세 번에서 13점 mapping·다섯 목표·무접촉·무잔류를 확인했다.
  조건과 수치는 [F02 기록](docs/stabilization-2026-11/F02_MAPPING.md)을 따른다.
- 공유 GUI simulator도 `0003`의 정수 epoch step과 mutex snapshot을 사용한다.
  기존 약 257초의 1 ns 불일치 원인과 GUI·수용 검증 범위는
  [F04 기록](docs/stabilization-2026-11/F04_GUI_CLOCK.md)을 따른다.
- 지도 외곽 fan/streak·전체 지도 정확도·다른 world/policy·장시간 안정성은 미검증이다.
- reset 뒤 입력 복구만 확인했으며 AMCL 재초기화·goal 재개는 미검증이다.
- 실제 로봇 동작, 최종 장착, physical goal, SDK 외 RL telemetry의 격리는 미검증이다.
- 새 OS rosdep provisioning과 GitHub hosted CI는 이번 AGX 결과로 검증되지 않는다.
