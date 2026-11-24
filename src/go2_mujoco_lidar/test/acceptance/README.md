# MuJoCo 수용 시험

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
필요하다. bridge는 현재 waypoint 수정 branch를 전용 prefix에 빌드한다.
모든 build는 새 `/tmp` 경로를 쓰며,
실행 output은 `--output`으로 지정한다.

AGX 프로젝트 루트에서 ROS/Unitree 환경과 W01의 `MUJOCO_ROOT`,
`UNITREE_MUJOCO_ROOT`를 지정한 뒤 실행한다. `RL_ROOT`는 controller object와
`config`·policy를 가진 저장소, `SDK_ROOT`는 Unitree SDK2 저장소다.

```bash
export RL_ROOT=/home/bi-agx1/go2_projects/projects/go2_rl_locomotion_baseline
export SDK_ROOT=/home/bi-agx1/go2_projects/shared/dependencies/unitree_sdk2
export BRIDGE_ROOT=/home/bi-agx1/go2_projects/projects/go2_nav2_mujoco_bridge
export A="$PWD/src/go2_mujoco_lidar/test/acceptance"
cmake -S "$A" -B /tmp/w07-acceptance-build -DCMAKE_BUILD_TYPE=Release \
  -DRL_ROOT="$RL_ROOT" -DSDK_ROOT="$SDK_ROOT"
cmake --build /tmp/w07-acceptance-build -j4
ctest --test-dir /tmp/w07-acceptance-build --output-on-failure
# bridge도 기본 install 대신 전용 prefix에 빌드한다.
colcon --log-base /tmp/w07-bridge-log build --base-paths "$BRIDGE_ROOT/src" \
  --build-base /tmp/w07-bridge-build --install-base /tmp/w07-bridge-install
source /tmp/w07-bridge-install/local_setup.bash
source /tmp/w07-close-install/setup.bash
export ROS_DOMAIN_ID=41 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$A/cyclonedds.xml"
export LD_LIBRARY_PATH="/tmp/w05-install/tf2-pin/tf2/lib:$LD_LIBRARY_PATH"
timeout --signal=TERM --kill-after=300s 1800s python3 -u "$A/acceptance.py" \
  --domain 41 --dds-config "$A/cyclonedds.xml" \
  --goals "$A/goals.json" --route "$A/route.json" \
  --simulator /tmp/w07-acceptance-build/acceptance_simulator \
  --controller /tmp/w07-acceptance-build/rlroot/deploy/robots/go2/build/go2_ctrl \
  --controller-cwd /tmp/w07-acceptance-build/rlroot/deploy/robots/go2/build \
  --world "$BRIDGE_ROOT/worlds/go2_complex_indoor.xml" \
  --tf-observer /tmp/w07-close-build/go2_mujoco_lidar/tf_owner_observer \
  --tf2-prefix /tmp/w05-install/tf2-pin --output /tmp/w07-run1
```

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
cmake --build /tmp/w07-close-build/go2_mujoco_lidar \
  --target planar_selection_broken_input -j2
/tmp/w07-close-build/go2_mujoco_lidar/planar_selection_broken_input \
  --gtest_filter=PlanarSelection.ChoosesClosestElevationAndFirstTie
```

## tf2 preload 종료 대조

새 shell에서 ROS·Unitree·변경 install만 source하고, 위 Domain 41/local DDS 환경을
설정한다. parent Python에 pin의 `LD_PRELOAD`·`LD_LIBRARY_PATH`를 추가하지 않는다.
다음 명령은 같은 실제 planner binary와 map/TF/clock/lifecycle 입력으로
pin 5회, system 5회, pin+deactivate/cleanup 선행 5회를 비교한다.
simulator·RL 또는 로봇 command publisher는 시작하지 않는다.

```bash
python3 -u "$A/teardown_probe.py" --cycles 5 \
  --pin /tmp/w05-install/tf2-pin/tf2/lib/libtf2.so --output /tmp/w07-tf2-ab
```

새 costmap 출력과 clock 역행 뒤 출력을 확인한 다음 SIGINT를 보낸다.
4초 안에 종료하지 않으면 hang으로 기록하고, 자신이 시작한 native child만
`gdb -batch -p PID -ex 'thread apply all bt'`로 최대 8초 동안 관찰한다.
child의 PR_SET_PTRACER 예외는 이 runner와 그 debugger에만 적용한다.
system ptrace 설정이나 기존 process의 권한은 바꾸지 않는다.
`result.json`은 각 trial의 실제 loaded library·hang·exit·잔류를,
`audit.json`은 기존 Domain 1/42 identity와 소유 session 잔류를 기록한다.
이 짧은 대조가 full SLAM/AMCL/action 환경을 대신하지 않는다.
