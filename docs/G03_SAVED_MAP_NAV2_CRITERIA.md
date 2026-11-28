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

## v2 사전 등록 (2026-11-28, v1 관찰 뒤)

v1은 `run-1`에서 **FAIL**로 판정됐다. 낮은 초기 기립 자세 834 frame과
Nav2 controller 1개·behavior 3개의 DDS endpoint가 두 항목을 실패했다.
v2는 이 결과를 본 뒤 작성했다. v1 기준·결과는 그대로 보존하며 v2의 통과로
v1 판정을 소급 변경하지 않는다. 아래 두 항목만 교체하고 나머지 v1 기준,
지도 hash·변환·목표·stack·설정·실행 횟수는 동일하다.
v2 기준은 새 실행 전에 독립 commit하고 구현은 다음 별도 commit으로 만든다.

### 기립 완료 뒤 낙상 0

기립 완료 시각 `t*`는 매 physics frame에서 base `z >= .22 m`,
`|roll| <= 15°`, `|pitch| <= 15°`가 연속 `1.0 sim s` 유지된 첫 구간의
**끝 시각**이다. 중간에 한 frame이라도 벗어나면 유지 시간을 다시 센다.
simulator 시작 `0 s`부터 `10.0 sim s` 안에 완료하지 못하면 FAIL이다.
목표 1 이전에 완료 receipt를 확인하며 늦은 기립으로 timeout을 덮어쓰지 않는다.
`t*`부터 simulator 종료까지 한 frame이라도 `z < .18 m` 또는
`|roll| > 60°` 또는 `|pitch| > 60°`이면 FAIL이다. 경계값은 기립의 경우
포함, 낙상의 경우 위 부등호 그대로다. 완료 시각·timeout·매 frame 자세를 보존한다.

- 자세 출처: world가 include하는 `unitree_robots/go2/go2.xml:285-287`,
  SHA256 `2014a3d76e30f17ab9447d8a67bd015291f74fa4d71ae30d005f1a32bd693d4b`의
  `home` qpos는 base `.27 m`, quaternion `(1,0,0,0)`인 직립 자세다.
- `.22 m`는 명목 home 높이 `.27 m`에서 사전 선택한 `.05 m` 자세 여유를 뺀
  기립 gate다. `.18 m`는 home 높이의 `2/3`이며 v1 하한을 유지한다.
  어느 값도 v1 trace 최저 높이에서 고르지 않았다.
- `15°`는 명목 직립 `0°` 주위의 사전 선택한 기립 허용각이다.
  `60°`는 v1의 큰 기울기 경계를 유지한다. 둘은 simulation 측정용 공학적
  선택이며 제조사가 보장한 자세·전도 한계가 아니다.
- 시간 출처: 형제 RL `deploy/robots/go2/config/config.yaml:40`의 FixStand
  `ts: [0,1,2]` 간격에서 유지 `1 s`를 선택했다. 기존 `acceptance.py`의
  `advance(3)` 기립 대기 3회분에 유지 1초를 더해 timeout `10 s`를 사전 선택했다.
  명목 home 자세와 기립 일정에 근거한 gate이며 v1 기립 완료 시각에 맞추지 않았다.

이 항목은 기립을 생략해도 합격하는 사후 구간 제외가 아니다. 기립 성공을
별도로 요구하고, 성공 이후에는 마지막 정리 physics frame까지 전도 손실을 검사한다.
접촉·clearance는 초기 기립도 포함해 v1처럼 전체 frame을 검사한다.

### 명령 소유권과 발신자별 중첩 0

주행 관측 구간은 localization ready부터 마지막 목표 terminal까지다.
command topic은 remap된 `/go2_nav2/shadow_cmd_vel`과 우회 경로 `/cmd_vel`이다.
PASS는 아래 조건을 모두 만족해야 한다.

- 관측 endpoint와 실제 메시지 발신자는 `/controller_server`, `/behavior_server`,
  `/go2_nav2_mujoco_bridge`만 허용한다. bridge는 현재 Twist publisher가 없지만
  계약상 허용한 구성원이다. 그 밖의 endpoint 또는 미식별 발신자는 FAIL이다.
- 서로 다른 **Nav2 publisher GID**의 수신 시각 간격이 `.25 wall s` 미만인
  메시지 쌍이 없어야 한다. 각 메시지의 `[수신 시각, 수신 시각+.25)` 창이
  다른 Nav2 GID의 창과 겹치면 FAIL이다. zero Twist도 포함한다.
  controller/behavior 서버 이름이 같아도 GID가 다르면 서로 다른 publisher다.
- 모든 endpoint 표본과 endpoint 수를 계속 기록하고 실제 command 관측이
  없거나 GID→endpoint 매핑이 누락되면 FAIL이다.

창의 출처는 형제 bridge
`src/go2_nav2_mujoco_bridge/go2_nav2_mujoco_bridge/bridge_node.py:35-36`의
`command_timeout_sec=0.25`다. 이 창은 명령이 bridge에서 유효하게 남는 시간을
측정하며 v1 endpoint 수 4개에 맞춘 수치가 아니다.

설치 Humble `rclpy.executors.Executor._take_subscription`은
`take_message` 결과의 message만 반환하므로 callback에서 publisher_gid를
얻지 못한다. RMW/rclcpp `MessageInfo`는 지원하므로 기존
`test/headless/tf_owner_observer.cpp`의 read-only 관측기를 확장한다.
각 Twist의 RMW publisher_gid·monotonic 수신 시각·sim 시각을 기록하고,
기존 Cyclone DDS builtin publication metadata의 `GID_MAP`으로 graph endpoint
GUID에 연결한다. action active/idle 추정 대신 실제 메시지 발신자를 측정한다.
한계는 관측기 수신 시각과 bridge 수신 시각의 scheduling 차이, DDS 전달 누락,
graph sampling 사이에 생겼다 사라진 미발행 endpoint다. 미식별 실제 발행은
합격으로 추정하지 않는다. `.25 s` 미만 전환도 사전 정의대로 실패로 센다.

정상 Nav2 서버가 미리 여러 publisher를 만들어 두는 구조와 실제 동시 명령을
분리한다. v2의 목적은 외부 command 주입과 command 유효 창의 충돌 검출이다.

### 실행 이후 고정

native·colcon·관련 tests·PR CI green 이후 fresh session에서 정확히 한 번 실행한다.
실행 시작 뒤에는 조건 변경·재시도가 없다. 실패 진단은 저장 로그만 사용한다.
소유자의 이번 승인에 따라 v2 PASS일 때만 PR #7을 저장소 관례대로 main에
병합하고 main CI green을 확인한다. FAIL이면 병합하지 않는다.

## v3 사전 등록 (2026-11-28, v2 관찰 뒤)

v1 `run-1`과 v2 `run-v2-1`은 모두 **FAIL**이며 기존 기준·로그·판정은
그대로 보존한다. v3는 **발신자 귀속 방법과 아래 fallback 규칙만** 바꾼다.
v2의 모든 threshold와 다른 criteria는 동일하다. 지도·변환·세 목표·stack·
설정·기립·전체 physics 관측·실행 횟수도 동일하다. 이 절은 구현보다 먼저
별도 commit하며 새 G03 실행 결과를 보기 전에 고정한다.

### 귀속 방법과 독립 관측

Primary는 기존 `rclcpp MessageInfo.publisher_gid`의 RMW publication handle을
Cyclone builtin publication instance handle → endpoint GUID → ROS graph
node로 연결한다. 24-byte 출력에서 RMW는 8-byte handle과 16-byte zero
padding이고 endpoint는 16-byte GUID와 8-byte zero padding이다.
이 두 표현을 byte 직접 비교하거나 임의로 16 byte로 자르지 않는다.
metadata는 `dds_take`로 소비하고 각 batch 뒤 loan을 반환하며, 남은 batch도
모두 배출한다. 256은 한 batch 크기일 뿐 전체 mapping 보존 상한이 아니다.
메시지 뒤에 도착한 metadata와 graph도 전체 기록에서 결합한다.

독립 채널은 `/follow_path/_action/status`와 behavior의
`/spin/_action/status`, `/backup/_action/status`,
`/drive_on_heading/_action/status`, `/wait/_action/status`,
`/assisted_teleop/_action/status`다. 실제 graph에서 추가 발견한 action status도
관측한다. steady 수신 시각·goal UUID·상태를 보존하고 ACCEPTED(1)·
EXECUTING(2)·CANCELING(3) 상태의 active 구간을 command 수신 시각과 대조한다.
이 채널은 controller/behavior의 역할과 active 구간을 독립적으로 설명한다.
상태 전달 순서와 terminal 직후 stop Twist 때문에 active 구간 밖 메시지도
별도 보고한다. 이 진단에 새 시간 threshold나 다른 goal 기준을 추가하지 않는다.

**Fallback 규칙: 미식별 primary sender를 면제하는 fallback은 허용하지 않는다.**
active action 하나만으로 sender GUID를 증명할 수 없으며 behavior 하나에도
여러 publisher가 있다. 따라서 단 하나라도 primary 귀속 불가 메시지가 있으면
FAIL이다. 상태 채널은 미식별 메시지를 합격으로 바꾸지 않는다.
외부 endpoint와 외부 sender는 계속 FAIL이다. 서로 다른 Nav2 publisher GID의
`[수신 시각, 수신 시각+.25 wall s)` 창 중첩은 계속 FAIL이다.
zero Twist를 포함하며 정확히 `.25 s`인 간격은 중첩이 아니다.

### 계측기 사전 증명과 단일 실행

AGX의 G03와 동일한 `rmw_cyclonedds_cpp`, Domain 41, acceptance loopback/tag에서
simulator 없이 알려진 command publisher와 graph GID로 먼저 계측기를 검증한다.

- (a) controller/behavior 두 publisher가 겹치지 않게 순차 발행: 올바른 귀속과 PASS.
- (b) 두 publisher가 `.25 s` 안에 고의로 발행: overlap을 검출하고 FAIL.
- (c) 추가 external publisher: endpoint·sender를 검출하고 FAIL.
- (d) 256개 초과의 publication이 있는 상태에서 늦게 생긴 command publisher와
  256개 초과 command publisher의 실제 메시지를 모두 귀속: 포화·오귀속 0.

실제 raw GID 양쪽 bytes·기대/관측 node·메시지 수·overlap·기대 verdict를 기록하고,
관련 unit test와 이 instrument 결과를 실행 전 branch에 commit한다.
native/colcon build·관련 tests·instrument test·정확한 HEAD의 PR CI가 모두 green인
뒤 fresh session `run-v3-1`에서 G03를 정확히 한 번 실행한다.
실행 중 한국어 `G03/` progress를 남긴다. 재시도·사후 기준 변경은 없다.
v3 PASS일 때만 PR #7을 병합하고 main CI green과 main hash를 확인한다.
FAIL이면 병합하지 않고 저장 로그로 진단한다.
