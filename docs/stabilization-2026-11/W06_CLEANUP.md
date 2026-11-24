# W06 정리와 LiDAR 기본값

W06의 변경 범위, 삭제 근거와 AGX 회귀 결과를 확인할 때 읽는다.
기준은 `cleanup/sim-stabilization`의 `a7e9c6c`다. 패키지는 합치지 않았으며
실물·비동작·합성 검증 경로는 계속 전체 workspace에 포함한다.

## 삭제와 공통화

삭제한 미사용 항목은 6개다: Python helper 파일 1개, 상수 4개, manifest 의존성 1개.
별도로 중복 종료 함수 1개와 전역 process-name marker 목록을 없애고, static TF의
report 타입 import 및 네 경로의 DDS 문자열/파일 판정을 공통 parser로 정리했다.

| 항목 | 변경 전 caller 검색 결과와 결정 |
| --- | --- |
| `go2_validation/typing_compat.py` / `assert_never` | 정의 외 import·호출 없음. `find_packages()`의 일반 Python 설치 대상일 뿐 console script나 launch caller가 없어 파일 삭제 |
| `mode_observer.MAP_SLAM_TF`, `MAP_AMCL_TF` | 정의 외 참조 없음. 실제 mode table은 `NO_GLOBAL_TF`, `MAP_FIXTURE_TF`, `SHADOW_MAP_SLAM_TF`를 사용하므로 두 상수 삭제 |
| `mapping_runtime_graph.RAW_CLOUD_QOS` | 정의 외 참조 없음. 삭제 |
| `mapping_runtime_graph.RAW_ODOMETRY_QOS` | `test_mapping_runtime_graph.py:19,30`의 depth assertion만 소비. 삭제하고 유지된 실제 observer의 `MAP_QOS`·`STREAM_QOS`를 검사 |
| `ros_gz_interfaces` | `go2_validation/package.xml` 외 source import·config·launch·CI 사용 없음. manifest 의존성 삭제 |
| `offline_process.stop_owned_process` | fault/live/shadow runner가 호출. 구현을 삭제하고 기존 `preflight_host.stop_owned_process`를 직접 재노출하여 caller API 유지 |
| `static_tf_profiles`의 report 타입 import | YAML 경계의 입력은 표준 `object`·`dict`로 검사하므로 report 모듈 import 제거. runtime 파싱의 값·검증은 유지 |

검색은 `typing_compat|assert_never|RAW_CLOUD_QOS|RAW_ODOMETRY_QOS|ros_gz_interfaces`
및 정확한 `MAP_SLAM_TF`·`MAP_AMCL_TF` 이름으로 수행했다. `src/` 전체의 import·launch·
setup entry point/data_files·CMake install, `.github/`, 형제 bridge 전체를 확인했다.
추적 README·runbook·문서 21개를 명시 경로로 별도 검색했고 해당 이름의 참조는 0개였다.
두 저장소의 README와 `setup.py`, LiDAR CMake도 직접 읽었다.
삭제 후 같은 source/CI/bridge 검색의 잔존 참조는 0개였다.

## 실제 caller가 있어 보존한 asset

| 항목 | 현재 caller / 보존 이유 |
| --- | --- |
| `nav2_shadow.yaml` | `go2_nav2_shadow.launch.py:19`, `go2_nav2_live_observer.launch.py:25`가 사용. 합성 fixture 및 실제 입력 no-goal observer 설정 |
| `shadow_open/blocked.yaml`, PGM | saved-map/shadow launch의 기본 지도, `go2_validation/config/shadow_scenarios.yaml`의 실제 scenario 지도. PNG는 README가 명시한 preview sidecar |
| 실패 BT | `shadow_action_runner._behavior_tree_path()`가 `planner_failure` goal의 `behavior_tree`에 지정 |
| 정상 BT | 새 sim localization과 기존 shadow/live Nav2가 모두 사용 |
| legacy localization / live observer / SLAM launch | live runner, nested launch와 bridge caller가 있음. bridge `go2_nav2_mujoco.launch.py:17`, runbook `:335`도 포함 |
| stage45/6/62 launch | 통합 sim bringup→stage45, stage6→stage45, stage62 SLAM→upright→stage45, bridge runbook `:263` caller와 진단 문서가 있음 |
| 실물 sensor/control/odometry/preflight 경로 | sim 기본 실행과 별개이며 실물·비동작 caller가 있어 유지. 9개 package 전체 build에 포함 |

`tf_owner_audit.py`는 runtime caller 없이 test가 사용하는 별도 계약으로 남아 있다.
`global_tf_owner_nodes()`의 이름 기반 추정도 유지했다. 이 두 검증 경계의 통합과
bringup 검증 모듈의 package 이동은 W04 이후 별도 작업이며 W06에서 완료로 주장하지 않는다.
기존 untracked 실험 자료는 이동·삭제·staging하지 않았다.

## 종료와 DDS 판정

`preflight_host.stop_owned_process()`가 launch parent에만 SIGINT를 보낸다.
parent가 이미 실패했더라도 같은 POSIX session에 실행 중인 child가 남으면 group
TERM/KILL로 정리한다. fault/live/shadow/preflight runner는 자신이
`start_new_session=True`로 만든 PID만 residual 검사에 전달한다.
다른 세션의 `map_server` 등 이름이 같아도 잔류로 세거나 signal을 보내지 않는다.

`bringup.dds_environment.cyclonedds_network_matches()`는 local file URI 또는
inline XML의 활성 Domain, 모든 interface와 loopback peer를 실제로 검사한다.
sim guard, runtime preflight, live mode observer, shadow 환경이 같은 함수를 사용한다.
mode별 domain·clock 정책 자체는 기존 계약을 유지한다.
잘못된 XML·없는 URI·다른 활성 domain·외부 peer·혼합 interface와 multicast 조건을
새 시험으로 확인했다. 실물 network에 연결하는 시험은 수행하지 않았다.

## LiDAR 계약과 외부 문서

모든 sim launch와 C++ node 기본값을 `720×8`, 수직 `±0.2617993877991494 rad`로
맞췄다. `/scan`은 기존 base-frame planar-selected cloud를 사용한다.
9/28 checkpoint의 조건을 기본값으로 선택한 것이며 geometry의 독립 정확도 우위를
새로 증명했다는 뜻은 아니다. 작은 16/64-ray 상태·오류 주입 시험의 단일 layer는
시험 oracle이므로 유지한다.

`go2_nav2/config/nav2_mujoco.yaml`, RotationShim과 최소 속도는 변경하지 않았다.
bridge 저장소도 수정하지 않았다. 외부 `XBOX_MANUAL_SLAM_RUNBOOK.txt`에서 바꿀 줄은
`:21`, `:34`, `:258`, `:266-268`, `:278-279`다. horizontal-only 설명을
720×8 raster와 base-planar 선택으로 바꾸고, 수직값은 `8/-0.2617993877991494/+0.2617993877991494`
또는 override 생략으로 바꿔야 한다. `:265`의 수평 720은 유지한다.

## 빌드·시험

- 변경 전 `a7e9c6c`: `/tmp/w06-before-{build,install,log}`, fresh Release 전체 9 package build exit 0.
- 변경 전 전체 colcon 집계: **228 tests, 0 errors, 3 failures, 2 skipped**.
- 기존 실패: rclpy fixture의 `ROS_LOCALHOST_ONLY=1`과 명시 lo XML 중복 1개, W05/F01의 sim executable·asset 추가를 반영하지 않은 package boundary 기대값 2개.
- 변경 후 `/tmp/w06-after-{build,install,log}`: fresh Release 전체 9 package build exit 0.
- 변경 후 전체 test와 docstring 수정 bringup 재시험의 최종 집계: **243 tests, 0 errors, 0 failures, 2 skipped**.
- 새 시험 15개: DDS XML 8개, owned session/강제 child 종료 2개, 실제 launch 생성 기본값 5개.
- 기존 skip 2개를 유지했다. 실패 시험을 삭제하거나 skip하지 않았다.
- 첫 변경 후 시험의 새 multiline docstring 진단 1개는 형식 수정 후 재시험해 해소했다.
- 강제 parent exit 7·TERM 무시 child의 KILL/reap, unrelated process 생존을 실제 subprocess로 확인했다.

빌드 환경은 ROS Humble와 기존 Unitree 메시지 overlay, W01 pin과 동일한 shared MuJoCo
3.3.6/root를 사용했다. 기본 `build/`, `install/`, `log/`는 변경하지 않았다.

## 수락 하네스에서 분리한 실패

첫 두 시도는 F01과 같은 중앙 8점 mapping 경로를 완료했지만 지도 저장 직전
`map→base` 조회가 과거 TF extrapolation으로 실패했다. run2에서 관찰기의 실제
tf2 pin을 확인해도 같았으므로 이 실패를 tf2 버전만으로 설명하지 않는다.

기존 B01 하네스의 `stop(route)`는 ros2-run parent 종료를 기다리는 동안 observer를
spin하지 않는다. W06 wrapper에서 종료 대기와 callback 처리를 분리한 run3은
그 대기에 실제 **15.316초**가 걸렸고, TF cache가 갱신되어 지도 저장과 localization,
다섯 goal 및 reset까지 통과했다. 경로·goal·판정 기준은 바꾸지 않았다.

run3의 원래 parent-only teardown은 planner PID `2211277` 하나를 남겨 마지막
assertion이 실패했다. 이 PID는 run3 owned 목록과 새 install argv를 확인한 뒤에만
종료했고 `/proc` 소멸을 확인했다. 원래 result의 실패 정보는 보존했으며 수동 정리는
`.omo/evidence/w06-cleanup/run3/owned-cleanup.json`에 별도로 기록했다.
최종 wrapper는 parent 종료 뒤 W06 공통 `stop_owned_process()`도 호출한다.

## 재현 명령과 증거 위치

AGX의 프로젝트 루트에서 실행한다. 각 재시험의 output은 기존 디렉터리와 겹치지
않는 새 경로여야 한다. simulator·RL driver와 tf2 pin은 F01에서 검증한 동일 파일이다.
simulator/ROS는 Domain 41, localhost와 B01 전용 discovery tag로 격리한다.

```bash
source /opt/ros/humble/setup.bash
source /home/bi-agx1/unitree_ros2/cyclonedds_ws/install/setup.bash
export UNITREE_MUJOCO_ROOT=/home/bi-agx1/go2_projects/shared/dependencies/unitree_mujoco
export MUJOCO_ROOT=/home/bi-agx1/go2_projects/shared/dependencies/mujoco-dist/mujoco-3.3.6
export ROS_DOMAIN_ID=41 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/go2_mujoco_lidar/test/headless/cyclonedds_w03.xml"

# before/after를 각각 독립 prefix로 실행했다.
colcon --log-base /tmp/w06-after-log build --base-paths src \
  --build-base /tmp/w06-after-build --install-base /tmp/w06-after-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+
source /tmp/w06-after-install/setup.bash
colcon --log-base /tmp/w06-after-log test \
  --build-base /tmp/w06-after-build --install-base /tmp/w06-after-install \
  --event-handlers console_direct+
colcon test-result --test-result-base /tmp/w06-after-build --verbose

source /tmp/f01b-install/local_setup.bash
source /tmp/w06-after-install/setup.bash
export CYCLONEDDS_URI="file://$PWD/.omo/evidence/b01-e2e/cyclonedds.xml"
export LD_LIBRARY_PATH=/tmp/w05-install/tf2-pin/tf2/lib:$LD_LIBRARY_PATH
timeout 1800 python3 -u .omo/evidence/w06-cleanup/acceptance.py \
  --output "$PWD/.omo/evidence/w06-cleanup/run4"
python3 .omo/evidence/w06-cleanup/audit.py
```

원시 결과·truth·지도·실제 LiDAR parameter dump·모드별 로그는
`.omo/evidence/w06-cleanup/run*/`에 있다. `acceptance.py`는 기존 B01 lifecycle과
F01 경로를 재사용하며 raster override를 제거한다. `audit.py`는 모든 시도의
owned PID 합집합, 기존 12개 PID/start tick/argv와 F01의 외부 source diff hash를
읽기 전용으로 대조한다.

## 최종 run4 결과

전체 acceptance는 exit 0이었다. 중앙 8점 mapping → 지도 저장 → mapping 종료 →
명시 initial pose의 AMCL localization → 동일한 다섯 world goal → reset 한 번을
수행했다. 실제 parameter dump에서 `720/8/−0.2617993877991494/+0.2617993877991494`를
확인했으며 raster override는 0개다.

| goal | world x,y m | 결과 | wall s | qpos 위치 오차 m | qpos heading 오차 rad |
| --- | --- | --- | --- | --- | --- |
| 1 | 0.8,0 | SUCCEEDED | 8.031 | 0.151507 | 0.133485 |
| 2 | 1.5,-1 | SUCCEEDED | 10.125 | 0.110047 | 0.093534 |
| 3 | 0,-1 | SUCCEEDED | 25.786 | 0.136695 | 0.112735 |
| 4 | -1,0 | SUCCEEDED | 23.205 | 0.104200 | 0.140689 |
| 5 | 0,0 | SUCCEEDED | 4.861 | 0.172678 | 0.106220 |

- 다섯 goal 모두 위치 오차 `≤0.3 m`를 통과했다. 위치·heading은 odometry와 독립적인 simulator freejoint qpos에서 계산했다.
- 지도는 `229×237`, resolution 약 `0.05 m`다. `run4/saved.png`를 직접 확인했고 중앙 자유 공간과 벽이 보이며 외곽 미탐색 영역과 fan/streak는 남아 있다.
- map/world 변환은 목표 전에 `(-0.01798655,-0.00979663,-0.00234305 rad)`로 한 번 고정했다.
- map→odom identity는 mapping에서 SLAM Toolbox 하나, localization에서 AMCL 하나였다.
- AMCL/Map Server/planner/controller/behavior/BT의 실제 tf2 6개와 Python 관찰기가 모두 `/tmp/w05-install/tf2-pin/tf2/lib/libtf2.so`를 로드했다.
- reset 뒤 cloud `257→261`, odom `286→290`, odom stamp `182 s→6.004 s`를 수신했고 localization·bridge child PID는 동일했다.
- 최종 run4 owned application PID 40개는 잔류 0개였다.
- 네 시도의 owned application PID 합집합 114개는 최종 감사에서 잔류 0개였다. 추가 owned argv 잔류와 FIFO 잔류도 0개였다.
- 기존 Domain 1 stack 12개의 PID/start tick/argv는 모든 시도의 전후 및 최종 감사에서 동일했다. 기존 process에는 signal을 보내지 않았다.
- RL과 공유 simulator의 HEAD·tracked diff SHA-256은 F01 감사와 같았다. bridge 저장소는 수정하지 않았다.
- `audit.py` exit 0이며 원문은 `.omo/evidence/w06-cleanup/audit.json`이다.

실물 동작, 전체 지도 정확도, 외곽 장거리 경로와 reset 뒤 AMCL 재초기화·goal 재개는
시험하지 않았다. 기존 Release planar assert oracle 문제(F20)는 W07 범위로 남긴다.
이 결과는 지정한 simulation world·정책·센서·goal과 입력 epoch 복구에 한정된다.
