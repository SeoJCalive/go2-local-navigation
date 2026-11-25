# W04 패키지 경계

현재 빌드 범위, 이동한 API와 외부 bridge의 호출 이관을 확인할 때 읽는다.
기준 HEAD는 `72e126a`이며 F01/W06의 runtime 설정을 유지한다.

## 목표와 실제 구조

| 패키지 | 책임 | workspace 직접 runtime 의존성 |
| --- | --- | --- |
| `src/description` | URDF·static TF/profile·DDS 환경 판정 | 없음 |
| `src/go2_inputs` | cloud gate·scan converter 조합·odom adapter | description |
| `src/go2_nav2` | SLAM/AMCL/Nav2 asset·session guard | go2_inputs |
| `src/go2_mujoco_lidar` | C++ sensor·sim 세션·지도 저장 | description, go2_inputs, go2_nav2 |
| `optional/real_robot/go2_sensor` | 센서 acceptance | 없음 |
| `optional/real_robot/go2_state_estimation` | source probe | go2_inputs |
| `optional/real_robot/go2_perception` | obstacle candidate | go2_inputs |
| `optional/real_robot/go2_control` | Sport 경계·trial recorder | 없음 |
| `optional/real_robot/bringup` | 실물·비동작 조합 | description, go2_inputs, go2_control, go2_perception, go2_sensor, go2_state_estimation |
| `tools/go2_validation` | fault/shadow/preflight observer·runner | bringup, description, go2_inputs, go2_control, go2_nav2, go2_perception, go2_state_estimation |

보고서 안 B의 canonical odom을 producer로 합치는 변경은 하지 않았다.
W02의 producer `/utlidar/robot_odom`과 adapter `/odom`이 W06 수용 경로이고,
이를 제거하면 이번 구조 정리에서 runtime 계약까지 바뀐다. 대신 기존 mapping과
odometry 입력을 `go2_inputs` 하나로 모아 core를 4개로 만들었다.
DDS XML parser도 가장 아래의 모델·환경 계약에 두어 검증 패키지 역의존을 끊었다.

## 이동과 보존

`git mv`로 모듈·launch·config·테스트를 이동했다. mapping gate는 실물 obstacle
node에서 QoS를 import하지 않고 공통 `go2_inputs.cloud_qos`를 사용한다.
QoS 값은 reliable/volatile/KEEP_LAST(1) 그대로다. source probe와 obstacle
candidate는 선택 패키지에서 core 계약을 재사용한다.

- static TF launch/profile/parser 및 DDS parser: bringup → description.
- cloud gate/scan parser/YAML/launch 및 adapter/continuity/epoch/YAML: → go2_inputs.
- preflight/fault/mode/TF-owner 모듈·observer·테스트: bringup → go2_validation.
- controller preview/costmap-only launch와 non-actuating YAML: go2_nav2 → bringup.
- 실물 패키지 5개: src → optional/real_robot; 검증 패키지: src → tools.
- synthetic maps·shadow/live launch·정상/실패 BT·stage45/6/62: W06의 실제 caller가 있어 보존.
- Nav2 sim YAML, RotationShim, 속도·오차·timeout·raster·topic/frame은 변경하지 않았다.

## 호출 이관

공개 `ros2 launch go2_mujoco_lidar go2_sim_bringup.launch.py`,
`go2_sim_navigation.launch.py`, `ros2 run go2_mujoco_lidar sim_save_map`과
`go2_nav2/config/nav2_mujoco.yaml` 경로는 그대로다.

| 옛 호출 | 새 호출 |
| --- | --- |
| bringup / go2_static_tf.launch.py | description / 동일 파일명 |
| bringup / go2_odometry_adapter.launch.py | go2_inputs / 동일 파일명 |
| go2_perception / go2_mapping_scan.launch.py | go2_inputs / 동일 파일명 |
| go2_perception / mapping_cloud_gate | go2_inputs / mapping_cloud_gate |
| go2_state_estimation / odometry_adapter | go2_inputs / odometry_adapter |
| go2_nav2 / go2_controller_preview.launch.py, go2_costmap_only.launch.py | bringup / 동일 파일명 |
| bringup / integrated_preflight_observer | go2_validation / 동일 executable |

Python API는 `bringup.preflight_*`, `fault_*`, `mode_observer`, `tf_owner_audit`를
`go2_validation`으로, `bringup.dds_environment/static_tf_profiles`를 `description`으로,
mapping/odom/continuity/epoch 모듈을 `go2_inputs`로 바꾼다.

## bridge 저장소에 필요한 변경

bridge repo는 수정하지 않았다. 기존 `go2_nav2` saved-map launch·BT·sim YAML과
stage45 launch는 유지하므로 이 소비자들의 package 이름 변경은 필요 없다.
`XBOX_MANUAL_SLAM_RUNBOOK.txt:287`의 수동 odom 호출만
`ros2 launch bringup go2_odometry_adapter.launch.py`에서
`ros2 launch go2_inputs go2_odometry_adapter.launch.py`로 바꾼다.
직접 static TF 또는 mapping scan을 실행하는 추가 caller가 있다면 위 표를 따른다.
bridge의 현재 package.xml은 go2_nav2를 직접 선언하며 bringup을 선언하지 않는다.
현재 launch의 직접 의존성은 유지되어 manifest 변경은 필요 없다.
core 4개만 빌드해도 기존 bridge의 saved-map/정상 BT lookup이 유지된다.
W05/W06이 이미 적은 Domain guard·설정 owner·raster 문서 이관은 별도 기존 항목이다.

## 검증 재현

`.omo/evidence/w04-boundaries/build_before.sh`와 `build_after.sh`는 기본 install을
사용하지 않고 `/tmp/w04-*`에서 실행한다. after는 전체 10개 패키지 build/test 뒤
core마다 별도 clean prefix에 `--base-paths src --packages-up-to PACKAGE`를 적용한다.
`tools/check_installed_core.py`가 각 prefix의 Python import·설치 launch 생성을 검사한다.
`tools/check_package_dependencies.py`는 runtime/test 선언을 구분하고 전체 그래프 순환,
Python import·literal launch/include/lookup·CMake workspace 참조를 검사한다.

회귀는 W06 `acceptance.py`를 복사해 observer module/prefix만 이관한 하네스를 사용한다.
mapping 경로·다섯 goal·0.3 m 판정·reset·owned cleanup은 바꾸지 않는다.
원시 로그와 결과는 `.omo/evidence/w04-boundaries/`에 보존한다.

## 빌드·정적 검사 결과

- 기준 `72e126a`: `/tmp/w04-before-{build,install,log}`의 fresh 전체 9개 build exit 0.
- 기준 전체 colcon 집계: 243 tests, 0 errors, 0 failures, 2 skipped.
- 변경 후 `/tmp/w04-after-{build,install,log}`의 fresh 전체 10개 build exit 0.
- 모든 runtime·manifest 수정을 마친 뒤 별도의 새 `/tmp/w04-final-{build,install,log}`에서도
  전체 10개 build exit 0, 59.2 s였다. `fresh-final.log`가 최종 clean build 근거다.
- 이동한 두 launch의 D213와 이동한 test의 E501은 bringup lint 범위에 새로 들어와
  첫 전체 시험에서 실패했다. 동작을 바꾸지 않고 docstring·줄바꿈을 수정했다.
- 최종 전체 집계: 246 tests, 0 errors, 0 failures, 2 skipped.
- 추가 3개는 정적 검사기의 정상 선언·누락 선언·순환 거부 oracle다.
  기존 테스트와 skip 수는 보존했다. 최종 재시험은 실패한 validation 경계와
  수정된 bringup에 한정했고 이전 통과 결과와 함께 집계했다.
- 정적 검사: 전체 10개 package 그래프 비순환, workspace 직접 의존성 누락 0.
  dynamic 참조 3개는 두 sim launch의 package 인자 lookup과 localization의
  외부 Nav2 node tuple이다. 실제 수용 시험이 사용하는 include와 설치 lookup을 확인한다.
- 네 core 각각 `/tmp/w04-sanity-PACKAGE-{build,install,log}`의 새 prefix에서
  `--packages-up-to` build exit 0. 선택된 package 수는 1/2/3/4다.
- 각 prefix에서 설치 Python import, 모든 core launch description 생성 및
  canonical URDF·mapping/config·share lookup이 통과했다.
- 이동으로 불필요해진 optional runtime manifest 의존성을 제거한 뒤 실물
  bringup/perception/source-probe 3개 재설치도 exit 0이었다.
- Python compileall과 `git diff --check`가 통과했다.
- W06과 이동한 runtime Python 80개를 AST로 비교했다. import·docstring·package/share
  소유권·소스 위치 변경을 정규화하면 함수 본문 차이는 0개였고 추출한 cloud QoS
  선언은 원본과 동일했다. C++ source/header와 sim Nav2 YAML의 Git diff도 0이었다.
  이 검사는 이관 검토 근거이며 실제 simulation 동작의 독립 oracle을 대신하지 않는다.
- rosdep check는 AGX rosdep이 초기화되지 않아 실행되지 않았다.
  시스템 설정은 변경하지 않았고 새 OS에 대한 외부 apt provisioning은 미검증이다.

`after.log`, `after2.log`는 최초 실패를 보존한다. 최종 집계와 독립 prefix 결과는
`final-checks.log`, 최종 optional 설치는 `optional-final.log`, 최종 의존성은
`dependencies.log`, rosdep 미실행 근거는 `rosdep.log`다.

## W06 동일 회귀 결과

`.omo/evidence/w04-boundaries/run1/result.json`과 `regression-run1.log`가 원시 근거다.
Domain 41, localhost, B01 전용 discovery tag의 새 simulator·go2_ctrl·FIFO를 사용했다.
W06과 같은 중앙 8점 mapping → map 저장 → mapping 종료 → AMCL 초기화 →
다섯 world goal → physics reset 한 번이 첫 수용 실행에서 exit 0이었다.

| goal | world x,y m | 결과 | wall s | 독립 qpos 위치 오차 m | heading 오차 rad |
| --- | --- | --- | --- | --- | --- |
| 1 | 0.8,0 | SUCCEEDED | 8.583 | 0.156385 | 0.073706 |
| 2 | 1.5,-1 | SUCCEEDED | 11.931 | 0.135581 | 0.080298 |
| 3 | 0,-1 | SUCCEEDED | 27.543 | 0.105733 | 0.139671 |
| 4 | -1,0 | SUCCEEDED | 20.102 | 0.036949 | 0.138012 |
| 5 | 0,0 | SUCCEEDED | 5.579 | 0.131163 | 0.121395 |

- 5/5가 `SUCCEEDED`이며 각 qpos 위치 오차가 `≤0.3 m`다.
- LiDAR parameter dump는 override 없이 `720×8`, 수직 ±0.2617993877991494 rad였다.
- 저장 지도는 `230×237`, resolution 약 0.05 m다. `saved.png`를 직접 확인했다.
  중앙 자유 공간·벽·기둥이 보이고 외곽 미탐색 영역과 기존 fan/streak는 남아 있다.
- mapping의 map→odom owner는 SLAM Toolbox 하나, localization은 AMCL 하나였다.
- Nav2/AMCL native 6개와 Python observer가 W06과 같은
  `/tmp/w05-install/tf2-pin/tf2/lib/libtf2.so`를 실제 로드했다.
- reset 뒤 cloud `263→267`, odom `293→297`, odom stamp `187→4.212 s`를 받았다.
  localization·bridge의 child PID는 같았으며 입력 epoch가 복구됐다.
- mapping/localization의 실제 시작 로그에 sensor·static TF·gate·scan·adapter·
  SLAM/AMCL/Nav2·session guard만 있었다. 실물 전용 node는 시작하지 않았다.
- owned application PID 40개, 추가 owned argv, FIFO의 최종 잔류는 모두 0개였다.
- 기존 Domain 1 stack 12개의 PID/start tick/argv는 작업 전후 동일했다.
  기존 프로세스에 signal을 보내지 않았고 기본 build/install/log는 변경하지 않았다.
- bridge·RL·공유 simulator의 HEAD와 tracked diff SHA-256도 전후 동일했다.
  `audit.py after`와 `process_audit.py after` 모두 exit 0이었다.
- 기존 미추적 실험 파일 988개는 staging·이동·삭제하지 않았다.

## 남은 범위

실물 동작, 전체 지도 정확도, 외곽 장거리 경로, reset 뒤 AMCL 재초기화·goal 재개는
검증하지 않았다. F20 Release planar assert oracle과 F22의 역사적 current manifest/
문서 정합은 별도 작업 범위다. W04는 package 경계·의존성·지정한 sim 회귀만 바꿨다.
