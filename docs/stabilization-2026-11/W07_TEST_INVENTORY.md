# W07 시험 inventory

e473c58과 W07의 네 core package 기본 시험을 비교한다. testcase는 parameterize를
펼친 pytest case와 GoogleTest case다. 기존 C++ main은 실행 파일 하나를 case 하나로
센다. CTest wrapper는 이 표에서 중복 가산하지 않는다. 전체 colcon 집계는
wrapper까지 포함하므로 이 표와 다른 수다.

동작은 순수 함수·OS/process·실제 ROS message·설치 launch 평가로 나누어 읽어야 한다.
설치 launch 평가는 실제 argument/condition/OpaqueFunction/typed parameter를
평가하되 node를 시작하지 않는다. configuration은 machine-consumed frame·plugin·
BT·sampling domain의 연결/유효성·주기와 deadline 관계 검사다. 숫자를 구현에서 복사한 정확한 기본값
검사는 별도 category다. source/prose 문자열 검사는 실행 의미를 증명하지 않는다.

| package | 동작/실제 평가 전→후 | machine config 전→후 | source/prose 전→후 | 복사 후보값 전→후 | Release 무효 oracle 전→후 | 합계 전→후 |
| --- | --- | --- | --- | --- | --- | --- |
| description | 11→14 | 0→0 | 1→0 | 1→0 | 0→0 | 13→14 |
| go2_inputs | 26→30 | 1→1 | 2→0 | 2→0 | 0→0 | 31→31 |
| go2_nav2 | 21→26 | 4→5 | 5→0 | 0→0 | 0→0 | 30→31 |
| go2_mujoco_lidar | 19→36 | 2→5 | 9→0 | 5→0 | 1→0 | 36→41 |
| 합계 | 77→106 | 7→11 | 17→0 | 8→0 | 1→0 | 110→117 |

## 파일별 전수 분류

`src/description/test/`:

- `test_dds_environment.py`: 8→8 동작. inline/file XML, 활성 domain, interface,
  loopback peer, malformed XML·URI, multicast.
- `test_static_tf_profiles.py`: 5→6. 전 동작 3/source 1/복사값 1.
  후 동작 6: allow/deny/unknown profile, canonical URDF의 radar_joint에서 직접 읽은
  xyz와 `qy=sin(pitch/2), qw=cos(pitch/2)` 비교, 실제 static TF launch의
  `use_sim_time` true/false typed parameter 평가.

`src/go2_inputs/test/`:

- `test_continuity_profiles.py`: 4→4. 전 동작 2/source 1/복사값 1.
  후 동작 4: malformed/unknown profile, observe/enforce 출력 차이,
  손계산 `sqrt(0.3²+0.4²)=0.5`를 caller limit 0.49/0.51 양쪽에서 검사,
  실제 adapter launch의 caller profile 전달.
- `test_mapping_cloud_contract.py`: 3→3 동작. point_step·row_step·data length의
  layout, finite count, timestamp/age. 입력 fixture는 wire-layout oracle다.
- `test_mapping_scan_configuration.py`: 5→5. 동작 3/config 1/source 1 →
  동작 4/config 1. converter/gate의 실제 생성과 boolean time 전달을 평가한다.
- `test_odometry_adapter_shutdown.py`: 5→5. 동작 4/복사 QoS depth 1 →
  동작 5. mock executor 예외·destroy/shutdown 횟수를 검증하며 real DDS 증명으로
  부르지 않는다. QoS는 source burst queue가 output보다 큰 관계를 검사한다.
- `test_odometry_contract.py`: 12→12 동작. frame·finite·stamp·회복·yaw·loss.
  후보 YAML을 oracle로 읽지 않고 caller-supplied ContinuityProfile fixture를 쓴다.
  recovery 2는 이 fixture의 입력이며 설치 default의 정확성 주장이 아니다.
- `test_simulation_epoch.py`: 2→2 동작. 실제 ROSClock 역행 callback 횟수와
  wall-time opt-out. 기본값 복사 없이 epoch sequence 자체를 oracle로 쓴다.

`src/go2_nav2/test/`:

- `test_live_navigation_observer_configuration.py`: 1 source → 1 실제 launch 평가.
- `test_localization_shadow_configuration.py`: 3→3. config 1/source 2 →
  config 1/실제 launch 평가 2. start_inputs=false가 두 include만 제거한다.
- `test_mujoco_rotation.py`: 1→2 config. D01 qpos 실측 0.362 rad/s와 Humble
  shim/iterator의 주기·signed slew·AND 의미를 기준으로 관계를 검사한다.
  0.5는 구현에서 옮긴 합격값이 아니라 D01 양방향 응답 범위다.
  BT action deadline은 `1/controller_frequency`보다 길어야 한다는 별도 관계를
  검사한다. 500 ms를 복사한 정확한 값 검사는 아니며 고의 20 ms 입력은 실패했다.
- `test_nav2_shadow_runtime_configuration.py`: 4→4. config 2/source 2 →
  config 2/실제 launch 평가 2. 생성된 remap과 lifecycle node_names를 읽는다.
- `test_sim_session.py`: 21→21 동작. Nav2 map schema·PGM magic, finite triplet,
  3-4-5 거리·±pi yaw, flock, TF2 source/version/hash tamper rejection.
  TF2 pin 버전은 third_party pin의 외부 dependency 계약이다.

`src/go2_mujoco_lidar/test/`:

- `test_raycast_error_budget.py`: 3→3 실제 ROS/process 동작. zero quaternion과
  identity 대조, K-1/성공/K 오류, 무효 0 budget. 로그 검사는 실행 event를 읽는다.
- `test_sim_session.py`: 19→20. guards/world 14 동작 유지. 복사 raster 5개를
  실제 launch declaration의 양의 정수·구면 elevation domain 유효성으로 대체했다.
  설치된 map-save의 `--help` 직접 실행 1개는 copy/symlink 양쪽 실행 권한을 검사한다.
- `test_stage45_integration_contract.py`: 1 source → 1 실제 launch 평가.
- `test_stage6_slam_contract.py`: ROS message 1/source 1 → ROS message 1/launch 평가 1.
  독립 MJCF imu site와 yaw +90°로 pose·body velocity를 계산하는 시험을 보존했다.
- `test_stage62_upright_hold_contract.py`: 9→4. source 7/config 2를 제거하고
  두 entry point의 caller raster/filter 전달 2, SLAM 단일 입력 소유 1,
  Domain 41에서 upright command-owner 생성 전 startup 거부 1로 대체했다.
  Domain 1 LowCmd motor/CRC 자체는 실행하지 않았으며 문자열 검사를 그 증명으로 남기지 않는다.
- `test_state_contract.cpp`: 1→1 executable 동작; 내부 explicit check 46개 유지.
  yaw/roll lever arm·body velocity·exact timestamp pair·duplicate·양방향 reset·bounded queue.
- `test_planar_selection.cpp`: Release에서 assert가 사라지는 1 executable →
  GoogleTest 4 cases. hand-computed quadrant/elevation/tie, self/floor/miss,
  empty/invalid input과 실제 known MJCF raycast. y=±2 box의 half-width 0.1에서
  sensor ±Y ray hit는 ±1.9 m다. canonical pitch는 Y축을 보존하므로 mount pitch를
  구현에서 복사하지 않는다. raw self/floor와 environment_only 제외도 실제 cast로 검사한다.
- `test_acceptance_processes.py`: 신규 2 실제 OS 동작. parent exit, orphan child,
  callback pump, unrelated process 생존, TERM 수락/무시의 signal 기록과 subreaper 회수.
- `test_sim_shutdown.py`: 신규 4 실제 service/OS/Launch 동작. manager 성공 뒤
  FINALIZED 확인, 성공 응답이지만 ACTIVE인 거부, 이미 FINALIZED인 manager 재호출 회피,
  입력 child에 SIGINT를 보내기 전 제품 shutdown hook 실행 순서와 child exit가
  다시 Shutdown을 발행해도 단회 실행되는 경계. 단회 옵션이 없는 음성 대조는
  실제 `managed, signal, managed` 순서로 실패했다.
  ROS service fixture는 protocol만 제공하며
  실제 Nav2 전이는 별도 full acceptance의 여섯 GetState record로 확인한다.

## 공유 helper와 수동 시험 경계

- `tools/launch_test_support.py`: 위 core 시험이 설치 launch를 평가하는 공통 helper다.
- `description.dds_environment`, `go2_inputs.simulation_epoch`는 위 8/2 cases에 포함된다.
- `test/acceptance/owned_processes.py`는 신규 OS case와 full simulator acceptance가 소비한다.
- `test/headless/`의 3개 manual verification script와 C++ simulator/GID observer는
  기본 testcase count에 포함하지 않는다. 기존 무중력/정지 시험의 주장 범위를 보존한다.
- 새 `test/acceptance/`는 일반 colcon test 밖의 manual 5-goal full simulation이다.
- pytest 108→112, native executable/case 2→5이며 CTest wrapper 5→8은 별도로 센다.
- `optional/real_robot`와 `tools/go2_validation`의 기존 131 pytest cases
  (22+10+6+0+8+85)는 범위 확장 없이 전체 colcon 회귀로 확인한다.
  그중 skip 2개는 기존 copyright 검사이며 W07이 추가하지 않았다.

## Release 음성 대조

`planar_selection_broken_input`은 정상 시험과 같은 source/oracle에서 ray 4의
입력 elevation만 -0.10에서 -0.50으로 바꾼 별도 target이다.
정상 선택 `{4,1,6,7}`을 기대하면 고의 변경 입력은 `{0,1,6,7}`을 내므로
GoogleTest가 exit 1이어야 한다. 정상 suite에는 등록하지 않는다.
명령은 [acceptance README](../../src/go2_mujoco_lidar/test/acceptance/README.md)를 따른다.
