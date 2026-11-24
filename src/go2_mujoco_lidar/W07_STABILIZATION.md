# W07 시험과 수용 진입점

네 sim core package의 시험 품질, F20의 Release oracle, 반복 실행 가능한
MuJoCo 수용 시험과 종료 경계를 확인할 때 읽는다. 기준은
`e473c58ce5d5d606f3debed2564bb1f971cb5995`이며,
파일별 분류는 [시험 inventory](W07_TEST_INVENTORY.md),
전체 simulator 실행 명령은 [수용 README](test/acceptance/README.md)에 있다.

## 기본 시험과 빌드

- 기준 source는 `git archive e473c58`로 `/tmp/w07-before-source`에 펼쳤다.
- 기준 전체 Release build: `/tmp/w07-before-{build,install,log}`, 10 packages, exit 0.
- 최종 source 전체 fresh Release build: `/tmp/w07-close-{build,install,log}`,
  10 packages, exit 0, 96초.
- 전체 경로는 `src optional/real_robot tools/go2_validation`이다.
- 기준 전체 colcon 결과: 246 tests, 0 errors, 0 failures, 2 skipped.
- 최종 전체 colcon 결과: 256 tests, 0 errors, 0 failures, 2 skipped, 93초.
- 실제 orphan/TERM/KILL 회귀 2개와 lifecycle service/Launch 회귀 4개가 포함된다.
- 외부 ament lint의 `SelectableGroups dict interface is deprecated` 경고 4개는
  기존 go2_sensor/bringup에서 발생했다.
- `tools/check_package_dependencies.py`, Python compileall, `git diff --check`가 통과했다.
- optional real-robot와 validation의 기존 131 pytest cases는 수정하지 않았다.

ROS/Unitree setup과 W01의 `MUJOCO_ROOT`·`UNITREE_MUJOCO_ROOT`를 지정한
AGX 프로젝트 루트에서 실행한다.

```bash
export ROS_DOMAIN_ID=41 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/go2_mujoco_lidar/test/headless/cyclonedds_w03.xml"
colcon --log-base /tmp/w07-close-log build \
  --base-paths src optional/real_robot tools/go2_validation \
  --build-base /tmp/w07-close-build --install-base /tmp/w07-close-install \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_ROOT="$MUJOCO_ROOT" -DUNITREE_MUJOCO_ROOT="$UNITREE_MUJOCO_ROOT" \
  --event-handlers console_direct+
source /tmp/w07-close-install/setup.bash
colcon --log-base /tmp/w07-close-test-log test \
  --build-base /tmp/w07-close-build --install-base /tmp/w07-close-install \
  --executor sequential --return-code-on-test-failure \
  --event-handlers console_cohesion+ --ctest-args -j1
colcon test-result --test-result-base /tmp/w07-close-build --verbose
```

core testcase는 parameterize를 펼치고 wrapper를 제외하면 110→117이다.
동작/실제 launch 평가는 77→106, machine config는 7→11,
source/prose는 17→0, 복사 후보값은 8→0, Release에서 사라지는 oracle는 1→0이다.
기존 C++ main 하나와 GoogleTest case를 서로 다른 단위로 세는 이유와
wrapper 중복을 포함하는 colcon 집계와의 차이는 inventory에 명시했다.

## F20의 실패 가능성

실제 Release target의 `flags.make`는 `-O3 -DNDEBUG -std=gnu++17`이었다.
정상 GoogleTest는 4/4 통과했다. 같은 oracle를 쓰는 별도
`planar_selection_broken_input`은 ray 4 elevation만 -0.10→-0.50으로 바꿨다.
`ChoosesClosestElevationAndFirstTie`의 결과 `{0,1,6,7}`가 손계산 기대값
`{4,1,6,7}`와 달라 1 FAILED TEST, exit 1이었다.
Debug에서도 정상 4/4와 고의 오류 exit 1을 확인했다.
고의 오류 target은 기본 CTest에 등록하지 않았다.
관찰한 Release 음성 대조 binary는
`/tmp/w07-final-build/go2_mujoco_lidar/planar_selection_broken_input`이다.

## 종료 원인과 수정 위치

W06 기록에서 B01의 `stop(route)`는 `ros2 run` parent 종료를 약 15초 기다리며
observer를 spin하지 않았다. TF cache의 10초 범위가 지나 저장 직전 조회가
extrapolation으로 실패했고, tf2 pin을 확인한 run에서도 재현됐다.
callback을 처리한 wrapper는 같은 경로를 통과했다.
또한 parent 종료만 확인하던 teardown은 owned planner child를 놓쳤다.
이 두 문제는 수용 하네스의 실행·대기·소유 추적 문제로 확인됐다.
별도로 run4·5의 native planner는 `global_costmap: Deactivating`에서 멈춰
launch가 SIGTERM 뒤 SIGKILL까지 사용했다. 무잔류 통과만으로 이 정지의 원인이
해결됐다고 판단하지 않는다.

`batch10`은 실제 native planner PID 2242840의 정지 후 2초에
`gdb -batch -p 2242840 -ex 'thread apply all bt'`를 최대 8초로 실행했다.
gdb exit 0으로 19개 thread를 얻었고, 원본은
`.omo/evidence/w07-acceptance/batch10/planner.stack`에 있다.

```text
T3: SignalHandler -> Context::shutdown -> on_rcl_preshutdown
    -> PlannerServer::on_deactivate -> Costmap2DROS::on_deactivate -> std::thread::join
T19: mapUpdateLoop -> updateMap -> getRobotPose -> lookupTransform
     -> tf2_ros::Buffer::canTransform -> nanosleep
```

같은 log의 planner deactivate는 1795536121.357초, clock publisher인
`mujoco_lidar_node`의 SIGINT는 1795536121.502초였다. publisher가 clean exit한 뒤에도
planner는 위 polling에 머물렀고 마지막에 SIGKILL이 필요했다.
[tf2_ros 0.25.23 Buffer](https://github.com/ros2/geometry2/blob/0.25.23/tf2_ros/src/buffer.cpp)의
timeout 조건은 `clock_->now() < start_time + timeout`이다.
[rclcpp preshutdown](https://github.com/ros2/rclcpp/blob/16.0.21/rclcpp/src/rclcpp/context.cpp)은
cleanup callback이 끝나기 전에 context를 invalid로 만들지 않는다.
따라서 reset 뒤 TF를 기다리던 map thread는 clock 입력이 먼저 끝나면
ROS 시간 timeout도 `rclcpp::ok()` 조건도 끝나지 않아 join을 막는다.
관찰된 stack은 tf2 mutex ABBA나 ABI layout 손상의 증거가 아니다.

제품 `go2_sim_navigation.launch.py`는 이제 cooperative launch Shutdown에서
Nav2 manager → localization manager를 먼저 SHUTDOWN한다.
`shutdown_managed_nodes()`는 별도 wall-time ROS context와 하나의 8초 deadline으로
모든 여섯 node의 GetState FINALIZED를 확인한 뒤에만 완료를 기록한다.
Humble launch는 handler를 앞에 등록하므로 이 synchronous hook은 마지막에 등록한다.
clock/TF process에 signal을 전달하기 전에 lifecycle cleanup을 끝내는 순서다.
helper가 실패하면 오류를 명시하고 launch의 나머지 process 정리는 계속한다.
runner는 six-node state record, 종료 오류 부재와 SIGKILL 부재를 별도 합격 조건으로 읽는다.
서비스 성공인데 state가 ACTIVE인 음성 대조와 실제 child의 READY→SIGINT 순서로
이 종료 경계를 검사한다. 터미널이 전체 process group에 직접 보내는 signal은
이 PID 대상 cooperative 경로와 구분하며 이번 full acceptance의 검증 대상은 아니다.
guard exit도 Shutdown을 다시 발행하므로 hook은 `handle_once=True`로 등록한다.
이 옵션이 없는 실제 child 회귀는 `managed, signal, managed` 순서로 실패했고,
수정 후에는 `managed, signal`만 관찰했다.

tracked runner는 bridge·route·map-save의 설치 executable을 직접 시작한다.
map-save도 소유 session으로 관리하고 완료를 기다리는 동안 ROS를 처리한다.
mapping 저장 pose는 새 odometry stamp에 해당하는 TF를 기다려 읽는다.
프로세스 생성은 별도 POSIX session으로 제한하며 종료 대기는 callback을
계속 처리한다. parent exit 후에도 session 전수를 검사하고 INT→TERM→KILL과
Linux subreaper로 descendant·zombie를 회수한다.

W07 첫 실행은 5/5·reset·무잔류를 통과했다. 정확한 stamp 대기를 추가한
두 번째 실행은 callback마다 `/proc` 전체를 반복 스캔하던 비용 때문에 TF 대기가
timeout됐다. 감사는 생성·종료 경계로 옮기고, 종료 중에는 executor가 준비된
callback을 bounded quantum 동안 처리한다. `/proc` glob 자체가 사라지는 PID를
따라가며 예외를 내던 경쟁도 숫자 directory 열거와 protected read로 고쳤다.
실패 run의 application 잔류는 0이었으나 감사 오류 때문에 판정을 실패로 보존했다.

## 수용 중 확인한 추가 원인 수정

fresh symlink install에서 map-save script의 source mode가 실행 불가였다.
copy install은 CMake의 PROGRAMS 권한을 받아 이 문제를 가렸다.
source executable bit를 수정하고 설치 경로의 `--help`를 직접 실행하는 시험을
추가했다. help 문구나 mode 숫자를 합격값으로 고정하지 않는다.

`clean1`의 G2는 planner goal acknowledgement timeout 뒤 BT halt에서
follow_path 결과를 받지 못해 ABORTED였고 qpos 오차는 0.957432 m였다.
BT 오류 1795538525.235초 뒤 controller의 취소 완료가 1795538525.297초에 나왔다.
[Humble 1.1.20 BT action server](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp)는
기본 `default_server_timeout`을 20 ms로 선언하고,
[BT halt](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp)는
같은 deadline으로 취소 결과를 기다린다.
[ControllerServer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp)는
WallRate loop마다 취소를 확인하므로 현재 10 Hz에서는 한 주기가 100 ms다.
simulation config에 scheduling 여유를 포함한 500 ms를 명시했다.
관계 시험은 `deadline > 1/frequency`를 검사하며 20 ms 고의 입력은 실패했다.
실제 응답 시간의 충분성은 아래 full runs로 따로 확인했다.

Humble Python Buffer는 clock 역행 callback을 스스로 등록하지 않으므로 observer는
공유 `watch_simulation_epoch()`로 cache를 비운다. reset 뒤 이전 epoch의
base transform을 계속 거부하던 TF_OLD_DATA를 이 관찰자 경계에서 해결했다.

## 동일 source 연속 전체 수용 세 번

원본 결과는 `.omo/evidence/w07-acceptance/verified{1,2,3}/result.json`이다.
세 번 모두 mapping/save를 통과했고 다섯 goal 각각 SUCCEEDED였다.
각 cell은 simulator freejoint qpos 기준 XY 오차(m)다.

| 실행 | G1 | G2 | G3 | G4 | G5 | reset 후 stamp(s) | 최장 stop(s) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| verified1 | 0.143949 | 0.107157 | 0.089308 | 0.106218 | 0.149609 | 0.008 | 4.956 |
| verified2 | 0.160793 | 0.129484 | 0.089271 | 0.058076 | 0.144768 | 0.124 | 4.520 |
| verified3 | 0.134440 | 0.072897 | 0.085900 | 0.065353 | 0.158297 | 0.042 | 4.951 |

- 모두 최대 허용 오차 0.3 m 이하다.
- 매회 새 cloud/odom을 4개 이상 관찰했고 두 입력 모두 위 stamp로 역행했다.
- localization/bridge session의 PID 집합은 reset 전후 동일했다.
- 매회 여섯 GetState가 FINALIZED(4), 종료 오류 0이었다.
- runner 송신은 SIGINT만 사용했고 launch 내부 TERM/KILL escalation도 0이었다.
- 매회 owned PID 39개, 잔류 0, 기존 Domain 1/42 보호 identity 21개 전후 동일이었다.
- 세 번 사용한 world hash는 모두
  `f164137d1b61d2c113df2e6f9fe9f0bb096f5c5085ec34547a2ed6e38ed39f2d`다.
- 이 full runs의 install은 `/tmp/w07-final-install`이다. 같은 source를
  이후 새 `/tmp/w07-close-*`로 전체 빌드하고 기본 suite 256개를 확인했다.

## tf2 preload 가설 대조

system package.xml은 0.25.23, W05 pin package.xml은 0.25.24였다.
같은 `/opt/ros/humble/lib/nav2_planner/planner_server`에 configure/activate,
실제 costmap 출력, clock 역행과 역행 뒤 출력까지 확인하고 SIGINT를 보냈다.
pin/system arm 사이의 차이는 preload와 library search path뿐이다.
각 native의 실제 `/proc/PID/maps`도 결과에 보존했다.

| 짧은 세션 조건 | pin 0.25.24 hang | system 0.25.23 hang | pin + lifecycle 선행 hang |
| --- | --- | --- | --- |
| static layer, 10 Hz update/publish | 0/5 | 0/5 | 0/5 |
| static+inflation, 5 Hz update/1 Hz publish | 0/5 | 0/5 | 0/5 |

두 번째 조건의 signal-first 종료는 pin 약 0.416–0.427초, system 약
0.417–0.428초였다. `tf2-ab/`와 `tf2-ab-inflation/`에 원시 결과가 있다.
모든 trial 잔류는 0, 기존 Domain 1/42 보호 대상 21개 identity는 전후 동일했다.
이 대조에서는 ABI preload 차이만으로 hang을 재현하지 못했다.
full SLAM/AMCL/action 환경에서의 ABI 문제까지 배제했다는 뜻은 아니다.

pin record의 commit은 `404b7224d623d614f18fa9738dbf1716403d857e`로
upstream 0.25.24 tag와 일치했다.
[0.25.23→0.25.24 diff](https://github.com/ros2/geometry2/compare/0.25.23...0.25.24)에
C++ header 변경은 없었다. `BufferCore::testTransformableRequests()`에서
transformable callback을 mutex 밖으로 옮긴 구현 변경과 버전·changelog 변경이다.
따라서 관찰된 정지를 C++ class layout 차이로 단정할 근거는 확인하지 못했다.
callback 실행 순서 변화와 종료 시 동시 실행은 별도로 확인해야 한다.
위 짧은 세션은 native가 끝날 때까지 parent가 clock/TF를 계속 발행하므로,
full launch에서 포착한 clock-first 종료 조건을 만들지 않는다.

## 실행 범위와 남은 경계

- 실행은 Domain 41, localhost, multicast off, `go2-w07-acceptance` discovery tag다.
- native simulator와 RL controller는 전용 `/tmp/w07-acceptance-build` binary다.
- 원본 W01 Go2Bridge와 RL FSM/policy object를 사용하고 native DDS 초기화만 감쌌다.
- bridge는 별도 저장소 `cleanup/domain-guard`의 `127a1925f2935fcf448fc369571e37bc8277d11f`를
  `/tmp/w07-bridge-{build,install,log}`에 빌드했으며 source는 수정하지 않았다.
- 외부 bridge의 최종 tracked diff는 비어 있었다. RL은 `a422ec4c`와 기존 tracked
  patch 3개, 공유 simulator는 `1eb6642e`와 기존 patch 2개를 사용했다.
  W07은 이 외부 source를 편집하지 않았다.
- observer와 localization TF2는 `/tmp/w05-install/tf2-pin/tf2/lib/libtf2.so`를 사용한다.
- 실제 Nav2 planner/costmap은 1.1.20, rclcpp는 16.0.21이었다.
- 기존 Domain 1 stack은 PID/start tick/argv를 전후 대조하며 signal 대상에 넣지 않는다.
- Domain 42에서 명령을 실행하지 않았다.
- 실제 로봇, 지도 전체 정확도, 외곽 장거리 경로, reset 후 AMCL 재초기화와 goal 재개는 미검증이다.
- 초기화 거부 시험은 legacy upright의 Domain 41 guard까지만 다룬다.
  Domain 1 LowCmd의 motor/CRC 동작을 source 문자열 검사로 증명하지 않는다.
- W06에 남아 있던 이름 기반 TF owner 추정·검증 경계 통합과 외부 runbook 수정은 W07 범위 밖이다.
