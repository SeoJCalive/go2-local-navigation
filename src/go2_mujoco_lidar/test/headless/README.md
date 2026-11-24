# W02 headless 상태 통합 시험

W05 지도 세션 시험은 아래 `verify_navigation_session.py`를 사용한다.
W03의 같은 Domain 41·전용 discovery tag와 정지 source를 재사용하며 주행 시험은 아니다.
현재 실행 순서와 외부 bridge의 nonzero-domain/loopback 계약은
[Nav2 세션 문서](../../../go2_nav2/SIM_SESSION.md)에 있다.

```bash
# /tmp/w05-install과 bridge install을 source하되 새 W05 overlay가 마지막이어야 한다.
# UNITREE_MUJOCO_ROOT, MUJOCO_ROOT, Domain/localhost/DDS는 W03 예와 같다.
bash third_party/tf2/setup.sh /tmp/w05
python3 src/go2_mujoco_lidar/test/headless/verify_navigation_session.py \
  --output "$PWD/.omo/evidence/w05-navigation-session"
```

W05 당시 이 시험은 설치된 bridge의 Domain 41 거부를 기록한 뒤 mapping, 지도 저장,
중복 모드 시작 거부, mapping 종료 후 localization, active AMCL에 초기 pose 1회,
AMCL 수락 및 Nav2 6개 lifecycle active를 확인한다. edge별 DDS publisher identity는
W03의 C++ observer로 확인하고 native child 6개의 실제 tf2 mapping을 보존한다.
기존 simulator/RL/default-install/bridge stack의 PID·start tick·argv는 전후 비교하며
시험이 시작한 PID만 종료한다. 새 실행은 기존 저장 지도 파일을 덮어쓰지 않으므로
재시험에는 새 output directory를 지정한다.

현재 bridge로 주행을 검증할 때는 [acceptance](../acceptance/README.md)를 사용한다.
위 W05 스크립트의 당시 거부 기대값은 현재 bridge의 수용 조건이 아니다.

MuJoCo의 실제 qpos와 DDS를 거친 cloud/odom을 비교할 때 사용한다.
GUI용 main 대신 작은 driver에서 W01에 고정된 `Go2Bridge::run()`과 `mj_step()`을
호출한다. simulator checkout과 header patch는 [W01 pin](../../../../third_party/unitree_mujoco/README.md)을
따른다. simulator source를 수정하지 않는다.

중력을 끈 freejoint를 yaw ±90°로 놓고 첫 1초 정지 후 world ±Y 속도 0.2 m/s를
설정한다. 각 6초 epoch 사이 `mj_resetData()`로 시간이 역행한다.
이는 실제 MuJoCo 센서·DDS·좌표·입력 복구 시험이며 RL/joystick 정책 주행 시험은 아니다.
모든 publisher/subscriber는 Domain 1과 localhost에 격리한다. 전용 Discovery Tag로
이미 실행 중인 Domain 1 세션도 분리한다. 물리 명령은 발행하지 않는다.
ROS의 loopback 선택은 `ROS_LOCALHOST_ONLY=1`이 맡고 SDK에는 driver가 `lo`를
명시한다. Humble RMW가 interface 설정을 앞에 추가하므로 공통 XML에 `lo`를 다시
나열하면 같은 interface 중복 선택으로 실패한다.

AGX에서 W01의 `UNITREE_MUJOCO_ROOT`와 `MUJOCO_ROOT`, ROS 및 새 install overlay를
지정한 셸에서 실행한다. SDK CMake prefix는 기존 AGX `/opt/unitree_robotics`다.

```bash
cmake -S src/go2_mujoco_lidar/test/headless -B /tmp/w02-headless-build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/w02-headless-build -j 4
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/go2_mujoco_lidar/test/headless/cyclonedds_w02.xml"
python3 src/go2_mujoco_lidar/test/headless/verify_runtime.py \
  --simulator /tmp/w02-headless-build/w02_headless \
  --lidar /tmp/w02-install/go2_mujoco_lidar/lib/go2_mujoco_lidar/mujoco_lidar_node \
  --model "$UNITREE_MUJOCO_ROOT/unitree_robots/go2/scene_terrain.xml" \
  --dds-config "$PWD/src/go2_mujoco_lidar/test/headless/cyclonedds_w02.xml" \
  --output "$PWD/.omo/evidence/w02-state-runtime"
```

`truth.csv`는 독립 base qpos, `observations.json`은 ROS 수신값, `result.json`은
epoch별 수신 수·중복 수·오차·정지 뒤 출력 수를 남긴다. 각 node 로그와 종료한
owned PID 목록도 보존한다. `mj_step()`은 qpos 적분 전 센서를 계산하므로 이동
중 센서가 qpos보다 1 step 뒤일 수 있다. 2 ms × 0.2 m/s = 0.4 mm와 DDS float32
양자화를 포함해 1 mm 이내를 검사한다. 이 오차 기준은 navigation 정확도 기준이 아니다.

## 2026-11-24 AGX 결과

- simulator checkout HEAD는 W01 pin `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`였다.
- 사용한 bridge header SHA-256은 W01 patch 적용본과 같은
  `fbcae26bc1b59a93023b6c8fc157ca2de8e93e2adb60bd84e9bac27ec00a1b51`이었다.
- `/tmp/w02-build`·`/tmp/w02-install`에 의존 패키지를 포함한 8개 새 Release build가
  성공했다. 최종 변경 LiDAR build도 exit 0이었다.
- state estimation Python 23개, perception Python 14개가 통과했다.
- LiDAR CTest 5/5가 통과했다. 내부 Python 12개와 C++ executable 2개이며,
  신규 state executable은 NDEBUG에서도 독립 기하·pairing·reset 검사 46개를 실행했다.
- colcon 집계는 CTest의 Python wrapper도 포함해 54 tests, 0 errors, 0 failures,
  0 skipped였다. 기존 planar executable의 Release assert 문제(F20)는 별도 미수정이다.
- headless integration exit 0이었다. 위치 표본 121개의 base qpos 대비 최대 오차는
  `0.0004000296754269428 m`였다.
- yaw ±90°에서 body-forward 표본 99개의 body x 속도는
  `0.1994348060~0.1996703199 m/s`, 최대 절대 body y 속도는 `0.000001896993 m/s`였다.
- raw cloud·source odom·validated cloud·canonical odom은 reset 전후 각각 60/61개였다.
- 네 stream의 동일 epoch/stamp 중복과 raw cloud/source odom stamp 불일치는 0이었다.
- LiDAR·cloud gate·odom adapter는 같은 PID로 유지됐고 clock 역행을 각각 한 번 기록했다.
- simulator 종료 0.5초 뒤부터 2초 관찰 종료까지 네 stream의 출력은 0이었다.
- 입력 정지 경고는 초기 입력 대기와 마지막 정지에 각각 한 번 기록됐다.
- 시작한 child PID 4개는 모두 wait/reap 뒤 `/proc`에서 사라졌다.
- 기존 Domain 1 simulator와 LiDAR는 건드리지 않았고 전용 Discovery Tag로 분리했다.
- 원시 결과·truth·관찰·로그는 `.omo/evidence/w02-state-runtime/`에 보존했다.

이 결과는 별도 headless driver의 무중력 자유 운동과 실제 pinned DDS bridge에 대한
관찰이다. GUI simulator main의 동시 실행 scheduling, RL/joystick 주행, SLAM/AMCL
지도 reset·재사용과 실제 로봇 동작은 검증하지 않았다.

## W03 통합 launch 검증

이미 실행 중인 Domain 1 stack의 binary를 교체하거나 DDS에 연결하지 않고 새 launch를
검사할 때 사용한다. W03은 `/tmp/w03-build`, `/tmp/w03-install`, `/tmp/w03-log`에만
build/test 결과를 쓴다. headless source는 W01의 실제 `Go2Bridge`와 `mj_step`을
호출하고 Domain 41·lo·전용 discovery tag를 고정한다. physics world는 새 install의
`scene/go2_indoor.xml`이다. 무중력 정지 상태로 SLAM 연결성을 확인하며 map 정확도,
RL 제어, 실제 로봇 동작은 검증하지 않는다.

ROS/Unitree overlay와 W01의 두 dependency root를 지정한 AGX 셸에서 실행한다.

```bash
colcon --log-base /tmp/w03-log build \
  --build-base /tmp/w03-build --install-base /tmp/w03-install \
  --packages-up-to go2_mujoco_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release
cmake -S src/go2_mujoco_lidar/test/headless -B /tmp/w03-headless-build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/w03-headless-build --target w03_headless -j 4
source /tmp/w03-install/setup.bash
export ROS_DOMAIN_ID=41 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/go2_mujoco_lidar/test/headless/cyclonedds_w03.xml"
colcon --log-base /tmp/w03-log test \
  --build-base /tmp/w03-build --install-base /tmp/w03-install \
  --packages-select go2_mujoco_lidar --event-handlers console_direct+
colcon test-result --test-result-base /tmp/w03-build/go2_mujoco_lidar --verbose
python3 src/go2_mujoco_lidar/test/headless/verify_bringup.py \
  --simulator /tmp/w03-headless-build/w03_headless \
  --tf-observer /tmp/w03-build/go2_mujoco_lidar/tf_owner_observer \
  --output "$PWD/.omo/evidence/w03-bringup-runtime"
```

Humble rclpy의 subscription API는 publisher GID를 callback에 제공하지 않는다.
따라서 test-only `tf_owner_observer`가 public rclcpp MessageInfo에서 edge별 GID를
TSV로 기록한다. Humble CycloneDDS는 이 sample GID에 publication handle을 넣고
graph endpoint GID에는 GUID를 넣으므로 두 값의 byte 직접 비교는 성립하지 않는다.
public DDS `DCPSPublication` metadata의 instance handle과 endpoint key로 둘을
연결한 뒤 Python observer가 graph endpoint와 대조한다.
관련 upstream 구현은 [rclpy subscription](https://github.com/ros2/rclpy/blob/humble/rclpy/src/rclpy/subscription.cpp)과
[Cyclone RMW message_info_from_sample_info](https://github.com/ros2/rmw_cyclonedds/blob/humble/rmw_cyclonedds_cpp/src/rmw_node.cpp)다.
고정 edge 기대 목록은 canonical URDF의 fixed joint에서 독립적으로 읽는다.
각 edge를 tf2 buffer에서도 직접 조회하고 `view_frames`의 PDF/YAML을 보존한다.
`/clock`, `/scan`, `/odom`, `/map`, 두 source state의 publisher 수와 모든 sim node의
`use_sim_time` parameter도 실제 graph에서 조회한다.
공식 converter가 내부 생성하는 `transform_listener_impl_*` helper는 parameter
service를 제공하지 않는다. [converter 구현](https://github.com/ros-perception/pointcloud_to_laserscan/blob/humble/src/pointcloud_to_laserscan_node.cpp)의
TF Buffer는 converter 자신의 `get_clock()`을 공유하므로, launch가 소유하는 converter의
`use_sim_time=true`와 그 buffer의 실제 transform lookup을 검사한다.

환경 matrix·실제 파일 XML parsing·`/proc` fixture가 guard/world unit test이고,
실제 LiDAR에 zero quaternion과 identity quaternion을 번갈아 입력하는 시험이 오류
예산 oracle이다. 예산 2와 3 각각에서 K-1 오류 후 성공은 살아 있고, 그 뒤 K번째
연속 오류에서는 exit 1인지 검사한다. 이는 의도적으로 주입한 무효 ROS state 시험이다.

`result.json`은 topic 목록·edge별 owner·수신 수·거부 결과·owned PID와 종료 후 잔류
목록을 보존한다. Domain 0 거부 시험은 ROS node 초기화 전에 launch에서 실패하므로
Domain 0 DDS participant를 만들지 않는다. simulator·검증용 ROS launch·GID observer
중 자신이 시작한 PID에만 signal을 보내며 기존 Domain 1 process는 다루지 않는다.

### 2026-11-24 W03 AGX 결과

- `/tmp/w03-*`에서 의존 패키지 포함 8개 fresh Release build가 exit 0이었다.
  GID observer 추가 후 변경 패키지의 최종 incremental build도 exit 0이었다.
- package Python test는 최종 29개가 통과했고 기존 C++ executable 2개도 통과했다.
  최초 오류 예산 test는 FATAL 요약문까지 오류 발생 횟수에 포함해 2개가 실패했다.
  실제 ERROR 행만 세도록 바로잡은 후 해당 target 3개가 모두 통과했다.
- active domain XML 검사 확장 후 해당 target 14개가 모두 통과했다.
  마지막 `colcon test-result`는 30 tests, 0 errors, 0 failures, 0 skipped였다.
  이 집계는 전체 pytest 29개와 최근 재실행 CTest wrapper 1개를 합한 값이다.
  C++ 2개와 이전 CTest wrapper 수를 중복 가산한 수치로 설명하지 않는다.
- installed `go2_sim_bringup.launch.py mapping:=true`의 headless 실행이 exit 0이었다.
  `/scan` 16개, `/odom` 20개, `/map` 1개를 관찰했다. map은 `67×116`, resolution
  약 `0.05 m`였다. 수신 수는 관찰 종료 시점의 값이며 SLAM 정확도 지표가 아니다.
- topic 목록에는 clock, raw/filtered/planar/validated cloud, scan, source/canonical
  odometry, map, tf, tf_static이 모두 있었다. 원시 목록은 `result.json`에 보존했다.
- `/clock`, `/scan`, `/odom`, `/map`, `/lowstate`, `/sportmodestate`의 publisher는
  각각 1개였다. clock owner는 LiDAR이고 map owner는 SLAM이었다.
- canonical URDF fixed 29개와 map/odom/base/LiDAR 연결 3개, 총 TF edge 32개가
  각각 실제 publisher identity 1개를 가졌다. DDS metadata와 graph GUID를 연결해
  owner를 확인했고 각 edge를 tf2 buffer로 조회했다. `view_frames`도 exit 0이었다.
- launch가 시작한 ROS node 7개와 GID observer의 `use_sim_time`은 모두 true였다.
  parameterless TF listener helper 2개의 이름도 별도로 기록했다.
- physics와 raycast world는 같은 resolved `go2_indoor.xml`이었다.
  시작 로그 SHA-256은
  `4cf00f56fad49701ea06c0c7fe01f0f4273063825929d3291cd50d3c8023231a`였다.
- Domain 0, loopback 누락, 다른 expected MJCF의 세 launch는 모두 exit 1로
  node 시작 전에 거부됐고 각 이유가 로그에 있었다.
- 시작한 장기 process/child PID 10개는 종료 후 모두 `/proc`에서 사라졌다.
  관찰한 기존 simulator·default-install·map_server/AMCL PID 6개의 argv와 시작 tick은
  전후 동일했다. 기존 process에 signal을 보내거나 default build/install/log를
  변경하지 않았다.
- 로그·topic/owner/parameter/result JSON·`view_frames` PDF/YAML은
  `.omo/evidence/w03-bringup-runtime/`에 보존했다.
- GUI·RL/joystick·SLAM reset/map 재사용·Nav2/AMCL 통합·실제 로봇 동작과 지도
  정확도는 검증하지 않았다. 기존 Release planar assert 문제(F20)는 W07 범위다.
