<!-- 목적: 독립 simulation 수용 절차. 고정 route/goal과 native 입력을 결과·잔여물
판정으로 연결한다. 실제 로봇 동작·전체 지도 정확도는 이 절차의 검증 범위 밖이다. -->
# MuJoCo 수용 시험

## G04 좁은 통로 진입점

`--saved-map <H3 wall-19.yaml> --narrow-passage --run-index <1|2|3>`은
[사전 기준](../../../../docs/G04_NARROW_PASSAGE_NAV2_CRITERIA.md)의 고정 42개
목표를 실행한다. 양성 34·음성 4·마지막 경계 4개이며 새 output으로 세 번
순차 실행한다. `narrow_passage.py`가 world/map 좌표·유한 목 단면 횡단·닫힌
틈 진입·음성 ABORTED/no-path·실제 recovery status와 sender를 판정한다.
기존 physics와 G03 v3 DDS 계측을 그대로 쓰며 `parameters.json`에 실제
global/local/planner parameter 전수를 기록한다. 경계 시작 시각 이후
성능/physics는 별도 기록하고 명령 계측·TF·종료는 전체 세션에 적용한다.
안전 표본은 새 CSV line만 읽어 관측 즉시 중지하며, 일반 goal 실패는 다음
목표로 진행한다. G04 progress는 run 번호·목표·시간·오차·최소 여유·회복·
경보·teardown을 `../live/progress.log`에 한국어로 남긴다.

## G03 저장 지도 진입점

`--saved-map`은 mapping과 reset을 생략하고 G03에 사전 등록한 H3 변환과 세
world goal을 사용한다. 같은 headless physics·RL·공개 localization launch와
Nav2 설정·FIFO bridge를 사용한다. 실행 전
[G03 기준](../../../../docs/G03_SAVED_MAP_NAV2_CRITERIA.md)을 먼저 읽는다.
기준 문서 commit 뒤 구현·build/test·PR CI green 순서이며 실행은 한 번뿐이다.

기존 아래 ROS 환경에서 다음 진입점을 사용한다.

```bash
python3 -u "$A/acceptance.py" --domain 41 \
  --saved-map "$PWD/.omo/evidence/g02-headless-repeatability/run-3/frozen/wall-19.yaml" \
  --output "$PWD/.omo/evidence/g03-saved-map-nav2/run-1"
```

이 mode에서 native driver는 `physics.csv`에 모든 physics step의 수평 collision
envelope 최솟값·base 위치·geom pair·fall을 기록한다. 기존 G02 analytic
clearance 함수를 재사용하며 기존 `contacts.csv`도 매 step 관측한다.
`truth.csv`는 독립 freejoint qpos다. `amcl_gt.csv`는 같은 stamp의 qpos 보간값을
고정 map frame으로 변환한 AMCL 오차다. bracket이 없으면 빈 오차로 남긴다.
`planned_paths.jsonl`은 `/plan`, `commands.csv`는 shadow command,
`result.json`은 goal별 실제 시간·오차·clearance 위치·AMCL 오차와 owner sample,
종료 여섯 상태와 residue를 담는다. `../live/progress.log`는 한국어 `G03/`
시작·goal 결과·contact/fall 최초 경보·종료 판정이다.

## 기본 mapping과 reset 수용

자신의 simulator·RL controller·FIFO bridge를 시작해 mapping → 지도 저장 →
localization → JSON에 지정한 N개 world goal → 입력 epoch reset을 재현한다.
수용 판정은 action `SUCCEEDED`, simulator freejoint qpos 기준 goal별 XY 오차
`≤0.3 m`, reset 뒤 새 cloud/odom·역행 stamp·같은 localization/bridge PID,
기존 process identity 보존과 owned session 잔류 0이다.
여섯 lifecycle node의 FINALIZED와 SIGKILL 미사용도 별도로 통과해야 한다.
F02에서는 전체 13점 mapping의 stall 0, 매 physics step의 비바닥 robot/world
접촉 0과 runner/launch 모두 TERM/KILL escalation 0도 판정한다.
실측 하한·경로 여유·긴 epoch clock과 연속 3회 결과는
[F02 기록](../../../../docs/stabilization-2026-11/F02_MAPPING.md)을 따른다.
기본 goals는 W04/W06과 같은 5개다. 지도 전체 정확도, 실물 동작 또는 reset 뒤
AMCL 재초기화와 goal 재개를 판정하지 않는다.

일반 `colcon test`에 등록하지 않는다. 기본 suite의 `test_acceptance_processes.py`와
`test_sim_shutdown.py`는 simulator 없이 OS·ROS service·Launch 종료 경계를 검사한다.
runtime ROS/SLAM/Nav2 설치,
W01 simulator 패치, 기존 RL policy/object build와 `cleanup/domain-guard` bridge가
필요하다. bridge는 현재 waypoint 수정 branch의 기본 install을 사용한다.
실행 binary와 library는 영구 경로에 준비하고 output은 새 `--output`으로 지정한다.

AGX 프로젝트 루트에서 실행한다. `SIM_DEPS` 기본값은 아래 영구 root다.
setup은 controller object·config·policy를 형제 `go2_rl_locomotion_baseline`에서
참조한다. 다른 checkout은 setup의 `RL_ROOT`로 선택한다.

```bash
# ROS overlay를 source하지 않은 clean shell
export SIM_DEPS="$HOME/.local/share/go2-local-navigation-sim"
BUILD_ACCEPTANCE=1 bash third_party/unitree_mujoco/setup.sh "$SIM_DEPS"
source /opt/ros/humble/setup.bash
source /home/bi-agx1/unitree_ros2/cyclonedds_ws/install/setup.bash
bash third_party/tf2/setup.sh
bash third_party/slam_toolbox/setup.sh
source third_party/slam_toolbox/install/local_setup.bash
export UNITREE_MUJOCO_ROOT="$SIM_DEPS/unitree_mujoco"
export MUJOCO_ROOT="$SIM_DEPS/mujoco-3.3.6"
df -h .
colcon build --packages-select go2_nav2 go2_mujoco_lidar \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/local_setup.bash
source ../go2_nav2_mujoco_bridge/install/local_setup.bash
export A="$PWD/src/go2_mujoco_lidar/test/acceptance"
export ROS_DOMAIN_ID=41 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$A/cyclonedds.xml"
export LD_LIBRARY_PATH="$PWD/third_party/tf2/install/tf2/lib:$LD_LIBRARY_PATH"
timeout --signal=TERM --kill-after=300s 1800s python3 -u "$A/acceptance.py" \
  --domain 41 --output "$PWD/runs/acceptance-01"
```

기본 simulator는 `$SIM_DEPS/acceptance-build/acceptance_simulator`, controller/cwd는
같은 build의 `rlroot/deploy/robots/go2/build/{go2_ctrl,}`다. GUI binary는 독립
truth/contact/reset CLI가 없어 이 driver를 대신할 수 없다.
tf observer는 현재 workspace install의 `lib/go2_mujoco_lidar/tf_owner_observer`,
tf2는 프로젝트 `third_party/tf2/install`, world는 형제 bridge의
`worlds/go2_complex_indoor.xml`이다. route/goals/DDS XML은 이 디렉터리의 추적 파일이다.
모든 path argument는 기존처럼 명시 override할 수 있다. launch의 `tf2_prefix`도
같은 영구 pin을 기본 선택한다. `/tmp` build 산출물을 요구하지 않는다.

새 output directory가 필수다. 재실행은 output만 바꾼다. domain을 바꾸면 XML의
Domain id와 환경도 같이 바꾼다. native driver는 같은 전용 discovery tag를 요구한다.
Domain 0과 병렬 진단용 Domain 42는 거부한다. simulator·RL은 ROS library path를
제거한 environment로 실행하며 원본 controller의 FSM·policy object를 재링크한다.
RL 설정의 3초 기립과 joystick pulse는 simulation-time 조건으로 기다린다.
action/lifecycle/route/TF/입력 복구는 해당 event와 bounded wall timeout으로 기다린다.

기본 `route.json`은 북쪽·남쪽·동쪽·서쪽을 왕복하는 원래 13점 coverage다.
동쪽 세 점만 `(5,-0.4) → (6.7,-0.4) → (5,-0.4)`로 옮겨 island를 피한다.
중앙 8점 비교는 `--route "$A/route_central8.json"`으로 선택한다.
`route_clearance.py --world <MJCF> --route "$A/route.json"`은 현재 world의
box/cylinder와 반지름 0.42 m footprint를 대조한다. 0.01 m 이하 간격의 표본에
반 간격을 더 빼 연속 직선 경로의 보수적 여유 하한을 보고한다.
이 값은 계획 중심선 기준이다. 실제 tracking 오차와 다리 움직임은
`contacts.csv`의 매 step 관측으로 별도 검증한다.

결과의 `criteria`는 항목별 boolean, `passed`는 종합 판정이다. `goals`는 action
status·world/map goal·qpos·XY 오차, `reset`은 역행 전후 stamp와 입력 수,
`shutdowns`는 session별 종료 시간·실제 보낸 signal·잔류를 담는다.
`shutdown_states`는 여섯 실제 GetState ID, `sigkill_logs`는 launch 내부 강제 종료 기록이다.
`shutdown_errors`는 lifecycle 종료 오류다. 무잔류여도 SIGKILL을 사용했거나
FINALIZED 확인이 빠지거나 lifecycle 종료 오류가 있으면 종합 판정은 실패다.
`preexisting.json`은 기존 Domain 1/42 PID/start tick/argv다. map YAML/PGM/sidecar, truth.csv, ROS/native
로그와 TF publisher identity도 같은 output에 남는다.
`mapping_wall_sec`, `map_shape`, `mapping_stalls`, `mapping_contacts`는 mapping
구간의 결과다. `nonfloor_contacts`는 reset을 포함한 전체 실행의 접촉 수이며,
`commands.csv`는 독립 ROS subscriber가 받은 속도 명령이다.

W06 종료 실패의 원인은 하네스가 `ros2 run` parent를 기다리는 동안 observer를
spin하지 않아 10초 TF cache가 낡아진 것과 parent 종료만 확인해 child를 놓친 것이다.
W07에서 callback마다 `/proc`를 훑던 감사도 callback 처리를 지연시켰으므로
감사는 생성·종료 경계로 옮겼다. 저장할 pose는 새 odometry stamp의 TF가
실제로 도착할 때까지 기다려 구시점 latest TF를 쓰지 않는다.
현재 runner는 설치 executable을 직접 시작하고, 종료 대기 중에도 callback을 처리한다.
자신이 만든 POSIX session만 INT → TERM → KILL로 제한 시간 내 정리한다.
subreaper가 고아 child를 회수하며 PID/start tick과 session 전수검사로 zombie도
잔류로 처리한다. 기존 process 이름을 기준으로 signal을 보내지 않는다.

native planner의 별도 정지는 signal-first 종료가 `/clock` publisher를 먼저 멈춰
costmap의 TF ROS-time timeout이 끝나지 않은 문제였다. 제품 session launch는
이제 Nav2 → localization lifecycle을 먼저 shutdown하고 FINALIZED를 확인한다.
runner는 launch PID 하나에 SIGINT를 보내 이 cooperative 순서를 사용한다.
hook은 guard exit가 Shutdown을 다시 발행해도 한 번만 실행한다.
observer는 공유 `watch_simulation_epoch()`로 TF cache를 비워 Humble Python
Buffer에 이전 epoch의 transform이 남지 않게 한다.
터미널 전체 group의 직접 signal은 이 경로와 다르며 수동 절차는
[SIM_SESSION의 종료 순서](../../../go2_nav2/SIM_SESSION.md#종료-순서)를 따른다.
stack과 preload 대조는 [W07 보고서](../../../../docs/stabilization-2026-11/W07_STABILIZATION.md)에 있다.

Release F20 음성 대조군은 다음과 같이 실행한다. 정상 binary와 같은 oracle을 쓰되
한 ray의 elevation 입력을 `-0.10 → -0.50 rad`로 고의 변경한 별도 binary다.
정상 suite에 등록되지 않으며 반드시 exit 1이어야 한다.

```bash
cmake --build build/go2_mujoco_lidar \
  --target planar_selection_broken_input -j2
build/go2_mujoco_lidar/planar_selection_broken_input \
  --gtest_filter=PlanarSelection.ChoosesClosestElevationAndFirstTie
```

## G03 저장 지도 v2

`--saved-map <H3 wall-19.yaml>`은 mapping·reset을 생략하고 고정 H3 변환과
세 world 목표를 사용한다. 판정과 출처는
[G03 사전 기준](../../../../docs/G03_SAVED_MAP_NAV2_CRITERIA.md)의 v2를 따른다.
기립 gate는 native `posture.hpp`가 매 physics frame에서 관측하고
`standup.json`의 완료 시각·timeout을 목표 1 전에 확인한다.
`physics.csv`는 기립 뒤 fall flag와 전체 구간 roll/pitch·clearance를 보존한다.
`acceptance_posture_test`는 simulator를 실행하지 않고 같은 gate의 경계를 검사한다.

기존 `tf_owner_observer`에 `observe_commands:=true`·`use_sim_time:=true`를
넘겨 command topic 둘의 RMW GID와 steady 수신 시각을 기록한다.
Humble rclpy가 버리는 sender 정보를 rclcpp/Cyclone builtin metadata로 연결하며
`command_owners.csv`와 `result.json.command_ownership`에 실제 발신자와
250 ms 창 중첩을 남긴다. endpoint 수는 `owner_samples`에 계속 보존한다.

G03 v3는 builtin publication을 `dds_take` batch로 배출해 늦게 생성된
publisher도 무제한 identity map에 보존한다. `ACTION_STATUS`는 실제 graph에서
발견한 action status의 UUID·상태·steady 시각이며, 결과의 active 구간과
메시지별 `active_actions`로 controller/behavior 역할을 독립 대조한다.
미식별 primary sender에 대한 fallback은 허용하지 않는다.
사전 기준은 [G03 기준](../../../../docs/G03_SAVED_MAP_NAV2_CRITERIA.md)의 v3 절이다.
같은 ROS/Domain/DDS 환경에서 아래 계측 검사를 먼저 수행한다. simulator와
RL은 시작하지 않으며, fresh output에 실제 raw bytes·graph GUID·네 verdict를 남긴다.

```bash
timeout 300 python3 "$A/command_instrument.py" \
  --observer "$PWD/install/go2_mujoco_lidar/lib/go2_mujoco_lidar/tf_owner_observer" \
  --output "$PWD/runs/g03-instrument-01"
```
이 관측기는 발행하지 않으며 bridge 수신 scheduling과 동일하다고 보장하지 않는다.

실행당 새 output이 필요하며 `.omo/evidence/g03-saved-map-nav2/run-v2-1`은
사전 승인된 v2 단일 실행 전용이다. 실행 뒤 조건 변경·재시도는 하지 않는다.

## GUI binary 장시간 clock 검증

F04의 `gui_clock.py`는 shared GUI binary를 Xvfb·software GL에서 실행한다.
별도 acceptance native main으로 대체하지 않는다. Domain 41·lo-only XML을 사용하고
ROS sensor bringup만 시작한다. 610초의 Sport·cloud·odom, 정수 ms 격자와
Low/Sport exact stamp 교집합, 출력 간격 1초 미만을 검사한 뒤 XTest Backspace로
reset하고 같은 ROS process의 5초 새 epoch를 기다린다.
Xvfb와 libXtst가 필요하다. shared binary의 동일 inode를 전용 output에 hard link하고
joystick 없는 pinned 기본 config를 복사하므로 공유 config와 기존 FIFO를 보존한다.

```bash
# ROS/Unitree와 현재 workspace install을 source한 셸
export ROS_DOMAIN_ID=41 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/go2_mujoco_lidar/config/cyclonedds_loopback.xml"
SHARED="$HOME/go2_projects/shared/dependencies"
timeout --signal=TERM --kill-after=60s 1400s python3 -u \
  src/go2_mujoco_lidar/test/acceptance/gui_clock.py \
  --simulator "$SHARED/unitree_mujoco/simulate/build/unitree_mujoco" \
  --config "$HOME/.local/share/go2-local-navigation-sim/unitree_mujoco/simulate/config.yaml" \
  --world "$PWD/../go2_nav2_mujoco_bridge/worlds/go2_complex_indoor.xml" \
  --output "$PWD/runs/gui-clock-01"
```

`ROS_LOCALHOST_ONLY=0`은 SDK와 RMW의 interface 중복 설정을 피하는 값이다.
실제 네트워크 경계는 위 XML의 `lo`, multicast off, `127.0.0.1` peer다.
결과는 `result.json`의 stream별 수·stamp 범위·최대 간격·reset epoch와 owned 잔류다.
실제 Xbox 조작·RViz·RL 주행과 지도 정확도는 소유자 GUI 시험으로 확인한다.

## tf2 preload 종료 대조

새 shell에서 ROS·Unitree·변경 install만 source하고, 위 Domain 41/local DDS 환경을
설정한다. parent Python에 pin의 `LD_PRELOAD`·`LD_LIBRARY_PATH`를 추가하지 않는다.
다음 명령은 같은 실제 planner binary와 map/TF/clock/lifecycle 입력으로
pin 5회, system 5회, pin+deactivate/cleanup 선행 5회를 비교한다.
simulator·RL 또는 로봇 command publisher는 시작하지 않는다.

```bash
python3 -u "$A/teardown_probe.py" --cycles 5 \
  --pin "$PWD/third_party/tf2/install/tf2/lib/libtf2.so" --output "$PWD/runs/tf2-ab"
```

새 costmap 출력과 clock 역행 뒤 출력을 확인한 다음 SIGINT를 보낸다.
4초 안에 종료하지 않으면 hang으로 기록하고, 자신이 시작한 native child만
`gdb -batch -p PID -ex 'thread apply all bt'`로 최대 8초 동안 관찰한다.
child의 PR_SET_PTRACER 예외는 이 runner와 그 debugger에만 적용한다.
system ptrace 설정이나 기존 process의 권한은 바꾸지 않는다.
`result.json`은 각 trial의 실제 loaded library·hang·exit·잔류를,
`audit.json`은 기존 Domain 1/42 identity와 소유 session 잔류를 기록한다.
이 짧은 대조가 full SLAM/AMCL/action 환경을 대신하지 않는다.
