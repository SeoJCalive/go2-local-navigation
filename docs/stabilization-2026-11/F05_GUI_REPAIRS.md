# F05 GUI 결함 수정

2026-11-25 AGX의 실제 GUI executable·Xvfb·Mesa·Domain 41·lo 격리로
G01의 RobotModel·저장 지도 표시·수동 종료 절차를 수정하고 확인했다.
제품 시험과 독립 시뮬레이터 qpos 시험은 실물 성능을 증명하지 않는다.

- RobotModel 원인: canonical URDF의 외부 `go2_description/dae`가 설치되지 않았고,
  기존 robot_state_publisher가 구독하는 `/joint_states`에 입력이 없었다.
- 수정: simulation은 canonical collision primitive를 visual로 사용한다.
  기존 LiDAR의 exact matched LowState motor q를 JointState에 발행한다.
- TF 소유권: 다리와 URDF fixed edge는 기존 robot_state_publisher만 소유한다.
  `odom → base`, `base → utlidar_lidar`, `map → odom` 소유자는 바뀌지 않는다.
- 순서 oracle: synthetic DDS 입력에서 12개 motor에 서로 다른 값을 넣고
  정확한 stamp·FR/FL/RR/RL 이름 순서·값을 수신하여 검증한다.
- GUI oracle: 여섯 세션의 실제 `/joint_states` publisher는 하나였고 네 발 TF가 연결됐다.

Humble MapDisplay의 `reset()`은 MFDClass reset 뒤 `clear()`를 실행한다.
clock 역행으로 RViz가 display를 reset하면 texture가 지워지지만 기존 subscription은
유지된다. Map Server는 activate 때 한 번 durable map을 발행하므로, 그 map 수신과
새 clock epoch의 순서에 따라 다음 sample 없이 빈 화면에 남을 수 있다.
`TransientMapDisplay`는 기존 MapDisplay를 상속하고 reset 때 구독을 새로 만들어
DDS historical sample을 다시 받는다. 새로운 지도·TF 발행자는 추가하지 않았다.

- 기본 RViz 설정: `src/go2_mujoco_lidar/config/simulation.rviz`.
- GUI 시작 순서: RViz 먼저 3회, Map Server 먼저 3회, 총 6/6 표시 성공.
- retained RViz: simulator/ROS를 새 epoch로 다시 시작한 추가 1회에서도 지도 재표시 성공.
- 화면·원시 로그: `.omo/evidence/g02-wall-hug/{shots,live,f05-*.log}`.
- GUI 시험 결과: 같은 디렉터리의 `f05-result.json`에 보존한다.

terminal Ctrl+C는 같은 foreground group의 모든 child에 SIGINT를 동시에 보낸다.
launch hook이 lifecycle service를 호출할 때 native child가 이미 종료되어
G01에서는 8.9초 뒤 `MANAGED_SHUTDOWN_FAILED`가 발생했다.
group을 유지한 상태에서 launch hook만으로 OS signal의 선행 전달을 되돌릴 수 없다.
정규 수동 종료는 다음 한 명령을 사용한다.

```bash
ros2 run go2_nav2 sim_stop --launch-pid "$ROS_LAUNCH_PID"
```

이 명령은 PID의 launch identity·domain·loopback을 확인하고 Nav2 → localization을
FINALIZED로 만든 뒤 launch PID만 SIGINT로 종료한다. simulator/RL/bridge는 명령
소유 대상이 아니며 ROS 종료 확인 후 각 terminal에서 종료한다.

- 단일 종료 명령: GUI 7회에서 여섯 FINALIZED를 확인했고 3.44~3.66초에 종료했다.
- 단위 시험: 134 tests, 0 errors, 0 failures, 0 skipped.
- 의존성 검사: 10 packages, 0 errors.
- 초기 단위 실행의 `UNITREE_MUJOCO_ROOT` 누락은 환경을 보완해 실패 사례를 재실행했다.
- GUI 시험 도구의 실행 mode 복사·갱신 전 install import 오류는 재시험 전에 고쳤다.
- GUI 종료 감사: owned 잔류 0, 원본 simulator config hash 유지, 기존 identity 유지.
- 추적 수락 하니스: 13점 route 완주 230.92초, stall 0, 비발바닥 접촉 0.
- Nav2: 5/5 SUCCEEDED, 독립 qpos XY 오차 0.195·0.127·0.125·0.102·0.139 m.
- reset: 같은 PID에서 cloud/odom stamp가 318초에서 0.142초로 바뀐 뒤 다시 수신됐다.
- 수락 종료: 여섯 lifecycle state 4, shutdown error 0, SIGTERM/SIGKILL 0, 잔류 0.
- 수락 원문: `runs/f05-acceptance/result.json`, 모든 12 criteria와 `passed`가 true.
- reset 경계의 관찰자에는 이전 epoch TF의 `TF_OLD_DATA` 경고가 남았고,
  recovery criterion과 stamp 변화·수신 재개는 별도로 통과했다.

G01 문서 오류는 SIM_SESSION과 bridge README·Xbox runbook에서 함께 교정했다.
LT 선행 입력, profile 축/버튼, RViz simulation time, 새 FIFO switch profile,
restart 뒤 Map 재확인, 영구 install/tf2 경로와 F04 clock 상태를 반영했다.
