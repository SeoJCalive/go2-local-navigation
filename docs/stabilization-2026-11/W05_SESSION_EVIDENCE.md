# W05/W06 세션 검증 기록

당시 source와 설치 binary의 관찰이다. 현재 절차는
[SIM_SESSION](../../src/go2_nav2/SIM_SESSION.md)을 따른다.

## W06 호출 조사 후 유지한 경로

- `go2_nav2_shadow.launch.py`, `nav2_shadow.yaml`, synthetic maps/실패 BT:
  `nav2_shadow_acceptance`와 실제 입력 no-goal observer의 caller가 있어 유지한다.
  sim 설정의 복제본으로 쓰지 않는다.
- 기존 `go2_nav2_live_observer.launch.py`, `go2_saved_map_localization.launch.py`,
  `go2_slam_mapping.launch.py`: 실물·검증 runner, 통합 sim bringup 또는 bridge caller가
  있어 유지한다. 패키지 이동은 W04에서 처리한다.
- `nav2_non_actuating.yaml`과 controller preview/costmap-only launch: 실물 또는
  비동작 시험 목적을 유지하며 W05에서 삭제하지 않는다.
- stage45/6/62 launch: 통합 sim bringup, nested stage launch, bridge runbook의 caller가
  있어 유지한다. bridge의 `nav2_mujoco.yaml`/Nav2 launch는 W06에서도 수정하지 않는다.

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
