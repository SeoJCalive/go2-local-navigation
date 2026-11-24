# MuJoCo mapping → saved-map localization 세션

시뮬레이션의 sensors·SLAM/AMCL·Nav2를 한 진입점에서 시작하고 저장 지도를
명시적으로 다시 선택할 때 사용한다. simulator, RL controller와 FIFO bridge는
각각 별도 process다. 모드 전환은 simulator를 reset하지 않고 ROS 세션만 재시작한다.
physics를 reset했다면 simulator/RL/ROS 세션 모두 새로 시작한다.

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

기존 bridge e3fe4a7은 Domain 1만 허용한다. **Domain 41의 이 절차에서 4단계는
현재 실행 불가**다. guard를 우회하거나 기존 Domain 1로 옮기지 않는다.
bridge의 후속 변경은 nonzero domain/실제 lo-only XML 검증 및 waypoint_runner의
같은 제한 해제다. bridge repo의 설정/launch 소유권 migration도 후속 작업이다.
이 프로젝트의 새 진입점은 Domain 0을 거부하며 실제 로봇 명령을 만들지 않는다.

별도 process의 명령 형식은 다음과 같다. `SIM_BIN`은 전용 W01 simulator build,
`WORLD`는 이 세션의 MJCF, `OWN_FIFO`는 그 simulator 설정의 전용 joystick device다.
공유 simulator의 config/FIFO를 바꾸거나 재사용하지 않는다. 현재 bridge 제한 때문에
아래 세 명령의 Domain 41 주행 조합은 미검증이며 마지막 명령은 거부된다.

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

아래는 전용 Domain 41 예다. simulator와 RL에도 같은 discovery tag가 필요하다.
ROS는 localhost interface를 RMW가 추가하므로 공통 XML에 interface를 중복 기재하지 않는다.
W03 headless 검증의 전용 XML을 재사용한다. 다른 세션이 쓰는 domain/tag는 선택하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source /home/bi-agx1/unitree_ros2/cyclonedds_ws/install/setup.bash
source /tmp/w05-install/setup.bash
export ROS_DOMAIN_ID=41 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/go2_mujoco_lidar/test/headless/cyclonedds_w03.xml"
bash third_party/tf2/setup.sh /tmp/w05

ros2 launch go2_mujoco_lidar go2_sim_navigation.launch.py \
  simulator_pid:="$SIM_PID" mode:=mapping tf2_prefix:=/tmp/w05-install/tf2-pin

# 별도 터미널, 같은 ROS/DDS 환경. output은 suffix 없는 새 절대 경로다.
ros2 run go2_mujoco_lidar sim_save_map \
  --simulator-pid "$SIM_PID" --output /tmp/w05-map/route

# mapping launch 종료/owned child 정리 완료 후 같은 simulator/RL을 유지한다.
ros2 launch go2_mujoco_lidar go2_sim_navigation.launch.py \
  simulator_pid:="$SIM_PID" mode:=localization \
  tf2_prefix:=/tmp/w05-install/tf2-pin \
  map:=/tmp/w05-map/route.yaml initial_pose:=0.0,0.0,0.0
```

initial_pose는 해당 지도의 현재 base 위치/방향이다. 위 원점은 형식 예이며 임의
spawn에 재사용할 값이 아니다. `INITIAL_POSE_SENT_ONCE`는 active AMCL과 subscription
확인 후 한 번만 기록된다. 그 뒤 새 `/amcl_pose`의 map frame, stamp, 거리 0.5 m와
yaw 0.5 rad window를 검사해 `INITIAL_POSE_ACCEPTED`를 기록한다. 이 window는
초기화 수락 조건이며 localization 정확도 합격 기준이 아니다. 수락된 후에만
navigation lifecycle을 시작한다. 90 wall seconds 내 수락/시작이 없으면 실패한다.

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

## W06 이관 검토 대상

- 기존 `go2_nav2_shadow.launch.py`, `nav2_shadow.yaml`, synthetic maps/실패 BT:
  software fixture 영역으로 이동할 후보이며 sim 설정의 복제본으로 쓰지 않는다.
- 기존 `go2_nav2_live_observer.launch.py`, `go2_saved_map_localization.launch.py`,
  `go2_slam_mapping.launch.py`: legacy/실물·검증 caller 조사 후 분리할 후보다.
- `nav2_non_actuating.yaml`과 controller preview/costmap-only launch: 실물 또는
  비동작 시험 목적을 유지하며 W05에서 삭제하지 않는다.
- stage45/6/62 launch와 bridge의 `nav2_mujoco.yaml`/Nav2 launch: 새 세션으로
  caller migration 완료 후 중복 소유권을 제거할 후보다. bridge는 W05에서 읽기 전용이다.

## 2026-11-24 AGX 검증 결과

- fresh Release build는 `/tmp/w05-build`, `/tmp/w05-install`, `/tmp/w05-log`에서
  8개 package 성공, exit 0이었다. 최종 변경 package 2개 설치 갱신도 exit 0이었다.
- private tf2는 위 고정 commit에서 별도 새 build, exit 0이었다. library SHA-256은
  `fa533e6985f95a45c319ec79ed470e844e97d1fca4ca809a8348acd9206a93d4`였다.
- 수정 package의 colcon 집계는 75 tests, 0 errors/failures/skipped였다.
  이는 Nav2 pytest 39개, LiDAR pytest 29개, C++ executable 2개와 pytest CTest
  wrapper 5개를 포함하는 집계다. 신규 지도/pose/flock/tf2 경계 pytest는 21개다.
  기존 Release planar assert 문제(F20)는 이 결과로 해결됐다고 주장하지 않는다.
- shell 문법, Python compile, `git diff --check`가 통과했다.
- Domain 41, localhost, W03 dedicated discovery tag의
  `verify_navigation_session.py`가 exit 0이었다. W03의 무중력 정지 simulator를 사용했다.
- mapping의 실제 map은 `67×116`, resolution `0.05000000074505806 m`였다.
- YAML hash는 `56793f7a22fa84dc42f2f0c5d0f7cebb472ccc5116bcfe39e8501fc47c97406d`,
  PGM hash는 `748c2922807e687fd7e3bb17a652fa0bb4465ef1de0d8360ed1c0c203a7b3d4a`였다.
- physics/raycast world hash는
  `4cf00f56fad49701ea06c0c7fe01f0f4273063825929d3291cd50d3c8023231a`였다.
- mapping이 살아 있을 때 같은 지도/pose의 localization launch는 lock에서
  exit 1로 거부됐다. mapping의 owned process를 종료한 뒤 localization이 시작됐다.
- 실제 map→odom publisher identity는 mapping에서 SLAM Toolbox 1개,
  localization에서 AMCL 1개였다. W03의 DDS handle→GUID→ROS endpoint 대조를 재사용했다.
- `/initialpose`는 `(0,0,0)`을 simulation time `19.398 s`에 정확히 1회 보냈다.
  AMCL은 `19.460 s` scan stamp로 `(0.0143483,-0.0183664,-0.000733156 rad)`를
  반환했다. 요청 XY와의 차이는 `0.0233066 m`다. 목표 주행 오차나 지도 정확도
  지표는 아니며, active AMCL이 초기화를 수락한 관찰값이다.
- Map Server/AMCL/planner/controller/behavior/BT의 lifecycle state는 모두 3(active)였다.
- SLAM PID `2191725`와 localization native PID
  `2191988,2192012,2192021,2192031,2192035,2192039`가 실제로
  `/tmp/w05-install/tf2-pin/tf2/lib/libtf2.so` 하나만 로드했다.
- 설치된 bridge는 Domain 41에서 `requires ROS_DOMAIN_ID=1`과 exit 1을 냈다.
  read-only 조건 때문에 수정/우회하지 않았다. RL 주행, scripted route, Nav2 goal
  2개, goal별 결과/ground-truth 최종 위치 오차는 미검증이다. 보낸 goal은 0개다.
- 기존 Domain 1 process 12개의 PID/start tick/argv가 전후 동일했다.
  `972935/74067749`, `973008/74068257`, `973116/74069001`,
  `973193/74069640`, `973199/74069640`, `973332/74070374`,
  `973334/74070374`, `973338/74070374`, `973340/74070375`,
  `973342/74070375`, `973344/74070375`, `973350/74070376`이다.
- 시험이 추적한 owned PID 35개는 정리 후 잔류 0이었다. 기존 process에는 signal을
  보내지 않았고 default build/install/log를 쓰지 않았다. bridge/RL repo도 수정하지 않았다.
- 원시 result JSON, YAML/PGM/sidecar, 모드별 로그와 TF identity는
  `.omo/evidence/w05-navigation-session/`에 보존했다. colcon 결과는 `/tmp/w05-*`에 있다.
