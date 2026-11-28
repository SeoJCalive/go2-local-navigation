# G03 저장 지도 Nav2 사전 등록 기준

2026-11-28 소유자가 승인한 simulation 한 번의 판정 기준이다.
이 문서를 구현과 실행보다 먼저 독립 commit한다. 실물 로봇은 사용하지 않는다.

- source: `f21af4de6744fe882a1b6e3712d479e77fd19504`.
- mode: 기존 acceptance headless physics → 동일 RL → 공개 localization launch
  → 기존 Nav2 → `/go2_nav2/shadow_cmd_vel` → 기존 0.25 s watchdog FIFO bridge.
- map: `.omo/evidence/g02-headless-repeatability/run-3/frozen/wall-19.yaml`.
- map YAML SHA256: `4dc279279c2b46bc4bc38b3b0ce5976724c8af822f8de5599533856f738b181f`.
- map PGM SHA256: `e7a33e7980fe55eb937f9f45af61c58683a7b0659651a43a0a6f6c086fa45514`.
- world: 형제 bridge `worlds/go2_complex_indoor.xml`.
- world SHA256: `f164137d1b61d2c113df2e6f9fe9f0bb096f5c5085ec34547a2ed6e38ed39f2d`.
- included Go2 MJCF SHA256: `2014a3d76e30f17ab9447d8a67bd015291f74fa4d71ae30d005f1a32bd693d4b`.
- policy: 형제 RL `logs/go2/go2_rl_lab/v0.1.8_fast_flat_40k/exported/policy.onnx`,
  SHA256 `100850a0b2cb463c7621f78fd273037af1f5a5c47cf9b2291c51a99b4307568b`.
- world→map: H3 frozen/result.json의 `saved_map_world_transform` 순서
  `(yaw, tx, ty) = (0.00006918148519170386, -0.0014251051562314404,
  0.000045958254625307515)`.
- 변환: `map_xy = R(yaw) * world_xy + (tx, ty)`이며 역변환은
  `world_xy = R(-yaw) * (map_xy - translation)`이다.
- 초기 world base XY/yaw: `(0, 0, 0)`. RL 기립 뒤 첫 localization pose는 그 시점
  독립 freejoint qpos에 위 고정 변환을 적용한다. 정지 기립 drift를 0으로 가정하지
  않으며 실제 초기 qpos와 AMCL initial pose를 결과에 기록한다.
- 목표는 world 좌표이며 순서와 yaw를 고정한다:
  `(0.8, 0, 0) → (1.5, -1, 0) → (0, 0, 0)`.

## 합격 조건

모든 항목을 만족해야 PASS다. 기록 누락은 합격으로 간주하지 않는다.

- NavigateToPose `3/3 SUCCEEDED`, 각 전송부터 terminal까지 wall time `≤150 s`.
- 각 terminal 시점 ROS와 독립적인 simulator freejoint qpos의 world XY 목표 오차 `≤0.30 m`.
- 모든 physics step에서 비바닥 robot/world contact `0`.
- falls `0`: 매 physics step qpos의 base `z <0.18 m` 또는
  `abs(roll)>60°` 또는 `abs(pitch)>60°`이면 fall frame으로 센다.
- 매 physics step collision-envelope clearance `≥0.10 m`.
- clearance 정의: 기존 `wall_hug/clearance_2d.h`의 `clearance2d`로 계산한,
  collision이 활성인 모든 robot geom과 비바닥 world box/cylinder의 수평 투영
  envelope 사이 signed gap의 최솟값이다. robot의 현재 자세·다리 형상을 포함한다.
  box는 wall 축에서 robot projected support rectangle을 사용하고 cylinder는
  world XY support rectangle과 원 사이 gap이다. 모서리에서는 보수적 하한이며
  음수만으로 실제 접촉을 뜻하지 않는다. meter, sim time, base XY, geom pair를
  매 step 기록한다. 전체 실행과 각 goal 구간의 최솟값·위치를 보고한다.
- command owner: 주행 구간 `/go2_nav2/shadow_cmd_vel` publisher가 매 관측에서 하나.
  controller/behavior의 정상 Nav2 역할 전환은 허용하되 동시 publisher 둘은 실패다.
- `map→odom` owner는 AMCL 하나이며 TF publisher identity로 확인한다.
- lifecycle 여섯 개 `planner_server/controller_server/behavior_server/bt_navigator/
  map_server/amcl` 모두 `FINALIZED`.
- forced kill과 TERM/KILL escalation `0`, owned process residue `0`,
  실행 전 기존 process identity 보존.

## 실행 고정과 관측

- 실행은 단 한 번이다. 실행 시작 뒤 조건·threshold·Nav2 params·footprint·inflation·
  bridge를 바꾸거나 실패를 재시도하지 않는다. 실패하면 저장 로그만으로 진단한다.
- footprint radius `0.42 m`, inflation `0.65 m`, resolution `0.05 m`와
  기존 `nav2_mujoco.yaml`, `saved_map_localization.yaml`, BT, bridge를 그대로 쓴다.
- mapping과 reset 시험은 이번 saved-map mode에서 수행하지 않는다.
- independent qpos, per-step clearance/contact/fall, AMCL pose와 같은 stamp의 보간 GT,
  planned path, command stream, owner samples, lifecycle shutdown과 residue를 남긴다.
- AMCL XY error는 같은 stamp의 GT에 위 고정 world→map 변환을 적용해 계산한다.
  AMCL error와 planned path는 진단값이며 별도 threshold를 추가하지 않는다.
- `.omo/evidence/g03-saved-map-nav2/live/progress.log`에 한국어 `G03/` 시작,
  goal별 성공/실패·시간·오차, contact/fall 최초 경보와 teardown verdict를 남긴다.
- 구현 단위 검사, colcon build와 관련 검사, PR CI가 green인 뒤 실행한다.
- PR은 main에 merge하지 않는다. 소유자가 결과를 보고 merge를 결정한다.
