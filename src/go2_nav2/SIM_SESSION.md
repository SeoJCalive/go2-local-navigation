# MuJoCo mapping → saved-map localization 세션

W04에서 내부 입력은 `go2_inputs`, static TF와 공통 DDS 판정은 `description`,
실물 후보는 `optional/real_robot`, 개발 검증은 `tools/go2_validation`으로 이동했다.
이 문서의 공개 sim 명령·Nav2 설정 경로·topic·frame·기본값은 유지한다.
현재 package layout은 [루트 README](../../README.md)를 따른다.
W07부터 전용 simulator/RL/bridge를 시작하는 반복 수용 시험은
[추적 하네스](../go2_mujoco_lidar/test/acceptance/README.md)를 사용한다.
현재 bridge의 `cleanup/domain-guard`는 nonzero domain과 loopback을 허용한다.
수용 시험도 형제 bridge와 이 프로젝트의 기본 install, 영구 dependency root를 사용한다.

시뮬레이션의 sensors·SLAM/AMCL·Nav2를 한 진입점에서 시작하고 저장 지도를
명시적으로 다시 선택할 때 사용한다. simulator, RL controller와 FIFO bridge는
각각 별도 process다. 모드 전환은 simulator를 reset하지 않고 ROS 세션만 재시작한다.
physics를 reset했다면 simulator/RL/ROS 세션 모두 새로 시작한다.

sim LiDAR의 기본값은 9/28 checkpoint와 같은 `720×8`, 수직 `±15°`다.
모든 sim launch와 직접 node 실행이 같은 raster를 사용한다. `/scan`은 기존
base-frame planar-selected cloud를 사용하며 sensor-frame 단일 수평 layer가 아니다.
일반 실행에서는 raster override를 생략한다. W06 호출 조사와 회귀 근거는
[W06_CLEANUP.md](../../docs/stabilization-2026-11/W06_CLEANUP.md)에 보존한다.

## 회전 명령과 정책 범위

`nav2_mujoco.yaml`은 Humble Nav2 1.1.20의 RotationShimController로 경로 초기
방향과 최종 방향을 맞춘다. shim은 signed `0.5 rad/s`를 내며 goal checker가
`0.15 m / 0.20 rad`에서 종료한다. 10 Hz에서 명령상 한 주기 회전량은
`0.05 rad`로 yaw 허용오차보다 작다. 경로 방향 진입/이탈 기준은 `0.35/0.10 rad`다.
shim의 command slew `20 rad/s²`는 반대 방향 odom `1 rad/s`에서도 한 주기 안에
`0.5 rad/s`를 요청하도록 한다. 실제 정책 가속도나 DWB rollout 가속도를 바꾸지 않는다.

D01 freejoint qpos 실측에서 제자리 `-0.5/+0.5` 입력의 응답은
`-0.362/+0.374 rad/s`다. 음의 `-0.3`은 `-0.014`에 그쳐 양방향 하한은
`0.5`로 선택했다. 기존 progress `0.20 rad / 10 s`는 유지한다.
최저 실측 회전율로도 0.20 rad에 약 0.56초가 필요하다.
DWB `XYThetaIterator::isValidSpeed`는 XY와 theta 최저 조건을 AND로 결합하므로
`min_speed_theta`만 올리는 대신 `min_speed_xy=0.05`도 함께 지정한다.
이는 느린/제자리 DWB 회전 후보에도 하한을 적용한다.

bridge는 Velocity_Down의 `v0.1.8_fast_flat_40k/params/deploy.yaml`에 맞춰
vx `[-2,4]`, vy `[-1,1]`, yaw `[-2,2]`를 정규화한다. 다른 policy 선택은
이 범위 계약을 다시 확인해야 한다. 위 수치는 MuJoCo 전용이며 실물 성능 근거가 아니다.

## 환경과 시작 순서

1. clean shell에서 자신이 관리하는 simulator를 `-s WORLD -i DOMAIN -n lo`로 시작한다.
   joystick device/FIFO도 이 simulator만 사용하는 경로를 선택한다.
2. clean shell에서 `go2_ctrl -n lo -d DOMAIN`을 시작한다. 연결 후 LT+A, 3초 기립,
   Start+D-pad Down 순서로 FixStand → Velocity_Down 진입을 확인한다.
3. ROS/Unitree와 선택한 local_navigation install을 source한 셸에서 아래 세션을 시작한다.
4. 같은 격리 DDS 환경에서 bridge의 설치된 `twist_to_joystick`만 실행한다.
   `-p fifo_path:=OWN_FIFO`를 지정한다. bridge의 Nav2 launch는 중복 실행하지 않는다.
5. mapping 주행 완료 후 지도 저장 CLI를 실행한다. ROS 세션을 Ctrl+C로 종료하고
   모든 owned ROS PID가 사라진 뒤 localization 모드를 시작한다.

이 프로젝트의 새 진입점은 Domain 0을 거부하며 실제 로봇 명령을 만들지 않는다.

별도 process의 명령 형식은 다음과 같다. `SIM_BIN`은
`$HOME/.local/share/go2-local-navigation-sim/unitree_mujoco/simulate/build/unitree_mujoco`,
`WORLD`는 이 세션의 MJCF, `OWN_FIFO`는 그 simulator 설정의 전용 joystick device다.
공유 simulator의 config/FIFO를 바꾸거나 재사용하지 않는다.

```bash
# 터미널 1: ROS overlay를 source하지 않은 simulator 전용 환경
"$SIM_BIN" -r go2 -s "$WORLD" -i 41 -n lo
# 터미널 2: ROS overlay를 source하지 않은 RL 환경
cd /home/bi-agx1/go2_projects/projects/go2_rl_locomotion_baseline/deploy/robots/go2/build
./go2_ctrl -n lo -d 41
# 터미널 3: 아래 ROS 환경과 mapping/localization launch
# 터미널 4: 같은 ROS/DDS 환경, bridge install을 추가 source
source /home/bi-agx1/go2_projects/projects/go2_nav2_mujoco_bridge/install/local_setup.bash
ros2 run go2_nav2_mujoco_bridge twist_to_joystick --ros-args -p fifo_path:="$OWN_FIFO"
```

아래는 GUI simulator/RL과 조합하는 전용 Domain 41 예다.
실제 lo-only XML을 사용하고 다른 세션이 쓰는 domain은 선택하지 않는다.
전용 discovery tag를 쓰는 headless/acceptance driver는 해당 하네스 XML을 별도로 따른다.

```bash
source /opt/ros/humble/setup.bash
source /home/bi-agx1/unitree_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=41 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/go2_mujoco_lidar/config/cyclonedds_loopback.xml"
bash third_party/tf2/setup.sh

ros2 launch go2_mujoco_lidar go2_sim_navigation.launch.py \
  simulator_pid:="$SIM_PID" mode:=mapping

# 별도 터미널, 같은 ROS/DDS 환경. output은 suffix 없는 새 절대 경로다.
ros2 run go2_mujoco_lidar sim_save_map \
  --simulator-pid "$SIM_PID" --output "$PWD/runs/gui-map/route"

# mapping launch 종료/owned child 정리 완료 후 같은 simulator/RL을 유지한다.
ros2 launch go2_mujoco_lidar go2_sim_navigation.launch.py \
  simulator_pid:="$SIM_PID" mode:=localization \
  map:="$PWD/runs/gui-map/route.yaml" initial_pose:=0.0,0.0,0.0
```

initial_pose는 해당 지도의 현재 base 위치/방향이다. 위 원점은 형식 예이며 임의
spawn에 재사용할 값이 아니다. `INITIAL_POSE_SENT_ONCE`는 active AMCL과 subscription
확인 후 한 번만 기록된다. 그 뒤 새 `/amcl_pose`의 map frame, stamp, 거리 0.5 m와
yaw 0.5 rad window를 검사해 `INITIAL_POSE_ACCEPTED`를 기록한다. 이 window는
초기화 수락 조건이며 localization 정확도 합격 기준이 아니다. 수락된 후에만
navigation lifecycle을 시작한다. 90 wall seconds 내 수락/시작이 없으면 실패한다.

## 소유자 GUI 시험

실제 Xbox 장치 입력과 RViz에서 보이는 지도·목표 주행을 확인할 때 사용한다.
자동 headless 수용 시험과 별개이며, 아래 육안·패드 항목은 소유자가 확인한다.
GUI 시험 전 simulator setup의 `0003-gui-integer-epoch.patch`를 적용한다.
이 패치는 GUI physics와 DDS snapshot을 같은 정수 epoch 시간에 맞추며,
재현·장시간 검증 근거는 [F04 기록](../../docs/stabilization-2026-11/F04_GUI_CLOCK.md)에 둔다.

1. simulator를 시작하기 전에 `/dev/input/js0`의 실제 Xbox 장치와 사용 중인 FD를
   확인한다. 전용 `unitree_mujoco/simulate/config.yaml`에서 `use_joystick: 1`,
   `joystick_type: "xbox"`, `joystick_device: "/dev/input/js0"`를 선택한다.
   bootstrap 기본값은 `use_joystick: 0`이다. 수동 패드에는 FIFO bridge를 실행하지 않는다.
2. 위 GUI simulator와 RL 명령을 clean shell에서 같은 nonzero domain·`lo`로 실행한다.
   LT+A → 약 3초 기립 → Start+D-pad Down을 확인한 뒤 mapping launch를 실행한다.
3. 같은 ROS/DDS 환경의 별도 셸에서
   `LD_LIBRARY_PATH="$PWD/third_party/tf2/install/tf2/lib:$LD_LIBRARY_PATH" rviz2`를 실행한다.
   Fixed Frame은 `map`, Global Options의 simulation time과 `/map`·`/scan`·TF를
   확인하고 Xbox로 실내를 주행한다. 지도와 scan/robot pose의 일치를 육안 확인한다.
4. 정지 후 위 CLI로 새 `runs/gui-map/route`를 저장한다. mapping ROS launch와
   owned child 종료를 확인하고 simulator/RL은 유지한다. 실제 현재 map pose로
   localization을 시작해 `INITIAL_POSE_ACCEPTED`와 `NAVIGATION_READY`를 확인한다.
5. Nav2 goal 주행에는 전용 FIFO로 새 simulator/RL 세션을 준비하고 그 FIFO의 bridge를
   같은 domain에서 시작한다. RViz의 `Nav2 Goal`로 목표를 순서대로 보내고
   action `SUCCEEDED`, 화면의 도착 자세와 장애물 여유를 확인한다.
6. 별도 시험 세션에서 physics reset 후 cloud/odom 새 epoch 복구를 확인한다.
   지도·AMCL·goal 재개는 보장하지 않으므로 전체 세션을 재시작한다.
7. 별도 셸에서 Domain 0 launch와 실제 simulator `-s`와 다른 `model_path`의
   bringup을 각각 실행해 node 생성 전 거부되는지 확인한다. 실제 world·domain은
   바꾸지 않는다. 종료는 아래 FINALIZED 확인 뒤 ROS → bridge → RL → simulator다.

## 종료 순서

localization의 costmap은 TF timeout에 ROS 시간을 사용하므로 `/clock` 입력보다
먼저 deactivate/cleanup되어야 한다. W07 자동 runner는 launch PID에만 SIGINT를
보낸다. 제품 launch의 cooperative Shutdown hook이 Nav2 → localization 순서로
SHUTDOWN하고 여섯 GetState FINALIZED를 확인한 뒤 입력의 종료를 진행한다.

직접 terminal에서 실행한 세션은 같은 ROS/DDS 환경의 별도 shell에서 아래처럼
먼저 lifecycle을 종료하고, 반환된 여섯 state가 4(FINALIZED)임을 확인한 뒤
원래 terminal의 Ctrl+C로 process를 종료한다. terminal이 전체 foreground group에
바로 보내는 SIGINT는 이미 모든 child에 전달되므로 launch hook의 순서와 다르다.

```bash
python3 - <<'PY'
from go2_nav2.sim_session_guard import shutdown_managed_nodes
print(shutdown_managed_nodes([
    ("navigation", ["planner_server", "controller_server", "behavior_server", "bt_navigator"]),
    ("localization", ["map_server", "amcl"]),
]))
PY
```

이미 모두 FINALIZED인 group은 SHUTDOWN을 다시 보내지 않는다.
Humble manager는 첫 SHUTDOWN 뒤 내부 lifecycle clients를 제거한다.
manager의 success만으로 종료를 판정하지 않고 각 node의 실제 state를 확인한다.
mapping은 기존처럼 종료하고 모든 owned PID가 사라진 뒤 다음 mode를 시작한다.

## 소유권과 provenance

- `map → odom`: mapping에서는 SLAM Toolbox, localization에서는 AMCL 하나다.
- 동일 domain의 새 진입점은 OS lock으로 직렬화한다. 이전 launch가 살아 있으면
  두 번째 모드를 시작하지 않는다. legacy 별도 SLAM/AMCL node도 runtime guard가
  이름으로 감시하며 반대 모드 또는 동일 이름 중복이면 세션을 실패시킨다.
- 다른 이름으로 직접 시작한 외부 TF publisher는 이 guard의 판정 대상이 아니다.
  격리 domain에서 session entry point 하나만 쓰며 TF edge identity는 runtime 시험으로 확인한다.
- `/clock`, `/scan`, `/odom`, static TF는 W03 bringup이 한 번씩 소유한다.
- 지도 YAML과 PGM은 필수이고 image는 YAML 기준 상대 경로로 해석한다.
  경로·SHA-256을 launch 로그와 저장 `.session.json`에 기록한다.
- 저장 sidecar에는 world path/hash, domain, mapping owner도 기록한다.
  `.posegraph/.data` 저장과 원자적 SLAM checkpoint는 이 CLI 범위가 아니다.
- tf2 pin은 [프로젝트 bootstrap](../../third_party/tf2/README.md)으로 준비한다.
  SLAM/map_server/AMCL/planner/controller/behavior/BT process마다 실제
  `/proc/PID/maps`의 `TF2_LOADED` 기록이 있어야 한다.
- Nav2의 sim 설정은 `config/nav2_mujoco.yaml` 한 곳이다. AMCL 설정은 기존
  `saved_map_localization.yaml`을 재사용하되 자동 원점 pose를 launch에서 끈다.

과거 실행 결과는 [W05/W06 기록](../../docs/stabilization-2026-11/W05_SESSION_EVIDENCE.md)에 있다.
