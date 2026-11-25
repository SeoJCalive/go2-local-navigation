# F03 영구 simulation 실행 경로

재부팅으로 `/tmp`가 비워져도 기본 install과 dependency root만으로 simulation을
시작하도록 바꾼 이유와 AGX 검증 결과를 확인할 때 읽는다.
실행 명령은 [루트 README](../../README.md)와
[수용 시험](../../src/go2_mujoco_lidar/test/acceptance/README.md)을 따른다.

## 경로 계약

| 대상 | 영구 기본 경로 |
| --- | --- |
| tf2 source/build/log | 프로젝트 `third_party/tf2/{build,log}` |
| tf2 pin | 프로젝트 `third_party/tf2/install` |
| GUI simulator | `~/.local/share/go2-local-navigation-sim/unitree_mujoco/simulate/build/unitree_mujoco` |
| 수용 simulator | 같은 dependency root의 `acceptance-build/acceptance_simulator` |
| 수용 controller | 같은 build의 `rlroot/deploy/robots/go2/build/go2_ctrl` |
| RL 원본 | 형제 `go2_rl_locomotion_baseline`의 기존 object·config·policy |
| TF observer | 프로젝트 `install/go2_mujoco_lidar/lib/go2_mujoco_lidar/tf_owner_observer` |
| bridge | 형제 `go2_nav2_mujoco_bridge/install` |
| 실행 output | 프로젝트 `runs/<새 실행 ID>` |

tf2 setup은 기본 인자 없이 위 prefix를 생성한다. 선택 인자는 작업 루트이며
`<root>/{build,install,log}`를 만든다. launch는 설치된 `go2_nav2` prefix에서
프로젝트 루트를 구해 tf2를 선택하고, 누락 시 setup 명령을 포함한 오류로 끝난다.
`tf2_prefix`와 `GO2_TF2_PREFIX` override는 유지한다.

수용 simulator는 GUI binary와 CLI가 다르다. 독립 freejoint qpos·매 step 접촉·
reset·전용 discovery tag가 필요하므로 기존 추적 driver를
`BUILD_ACCEPTANCE=1 bash third_party/unitree_mujoco/setup.sh`의 선택 산출물로 만든다.
controller도 원본 FSM/policy object는 유지하고 동일 DDS 격리 adapter만 재링크한다.
수용 runner의 모든 path argument는 계속 override할 수 있다.

tf2 산출물과 runs는 Git에서 제외한다. `third_party/tf2/COLCON_IGNORE`로
private geometry2 source를 일반 workspace package 탐색에서 제외한다.
수동 Xbox·GUI·RViz·지도 저장·localization·목표·reset·Domain 0·다른 world의
점검 절차는 [SIM_SESSION](../../src/go2_nav2/SIM_SESSION.md#소유자-gui-시험)에 둔다.

## 빌드와 자동 검사

- native 수용 driver Release build: exit 0.
- 실제 MuJoCo 두 epoch clock/음성 대조: 1/1 passed.
- tf2 `0.25.24` 고정 commit과 library receipt 생성: exit 0.
- 기본 install의 `go2_nav2`, `go2_mujoco_lidar`만 Release 재빌드: exit 0.
- 두 패키지 colcon 결과: 85 tests, 0 errors, 0 failures, 0 skipped.
- 누락 tf2가 bootstrap 명령을 보고하는 회귀 시험 포함.
- 설치된 launch `--show-args`: tf2 기본값이 프로젝트 영구 prefix.
- setup 셸 문법·변경 Python compile·`git diff --check`: 통과.
- 각 build 직전에 `df`를 확인했다.

## 영구 기본값으로 한 번의 수용 실행

2026-11-25 AGX에서 기본 workspace/bridge install과 영구 dependency만 source했다.
runner에 준 인자는 `--domain 41 --output "$PWD/runs/f03-durable-01"`뿐이다.
ROS는 localhost-only, native는 `lo`, 모두 추적 XML의 전용 discovery tag로 격리했다.
`GO2_TF2_PREFIX`를 unset해 launch 기본값을 실제로 사용했다.
1800초 bounded command가 runner 종료와 최종 `result.json`의 `passed`를 확인했다.

| 판정 | 결과 |
| --- | --- |
| mapping | 13/13점, 232.239초, stall 0 |
| 지도 저장 | 318×240, 0.05 m/cell, YAML·PGM·session sidecar |
| goal 1 | SUCCEEDED, qpos XY 오차 0.204778 m |
| goal 2 | SUCCEEDED, qpos XY 오차 0.102737 m |
| goal 3 | SUCCEEDED, qpos XY 오차 0.129926 m |
| goal 4 | SUCCEEDED, qpos XY 오차 0.121339 m |
| goal 5 | SUCCEEDED, qpos XY 오차 0.146071 m |
| reset | cloud/odom 각각 3064→3068, stamp 325.302→0.182초, 같은 PID |
| 종료 | 여섯 lifecycle FINALIZED=4, TERM/KILL escalation 0, owned 잔류 0 |
| 접촉·기존 process | 비바닥 접촉 0, 기존 identity 보존 |
| 종합 | `passed: true`, 12개 criteria 모두 true, command exit 0 |

`result.json`의 `paths`는 실제 선택한 영구 경로를 보존한다.
observer와 여섯 localization/Nav2 native child도 프로젝트 pin의 library를 로드했다.
실행 전 AMENT/LD_LIBRARY/PYTHON 경로에 `/tmp/`가 없음을 검사했다.
원시 결과는 Git 제외 `runs/f03-durable-01/`, 실행 console은
`.omo/evidence/f03-durable/acceptance-console.log`에 남는다.
reset 후 이전 epoch TF에 대한 경고는 있었으며 입력 복구·종료 기준은 모두 통과했다.

## 승인된 임시 디렉터리 정리

수용 통과 후 본인 소유의 실제 디렉터리만 선택하고 `/proc`의 cmdline·environ·maps·
cwd/exe/root/FD와 `lsof +D`를 검사했다. 발견된 참조는 없었고 lsof의 stdout/stderr도
비어 있었다. 일부 process의 maps/FD 읽기 제한은 감사 목록에 그대로 남겼다.
`sudo` 권한이나 비공개 process 내부를 별도로 검증하지는 않았다.

아래 brace 표기는 삭제된 전체 88개 디렉터리를 정확히 묶은 목록이다.

- `/tmp/b01-{build,install,log,rlroot,sim-build,test-log}`
- `/tmp/f01-{build,install,iterator-build,log}`
- `/tmp/f01b-{build,install,log}`
- `/tmp/f02-{acceptance-build,bridge-build,bridge-install,bridge-log}`
- `/tmp/w02-{build,final-log,final2-log,final3-log,headless-build,install,lidar-test-log,lidar-test2-log,lidar-test3-log,lidar-test4-log,log,test-log}`
- `/tmp/w03-{build,headless-build,install,log}`
- `/tmp/w04-{after,before,final}-{build,install,log}`
- `/tmp/w04-sanity-{description,go2_inputs,go2_mujoco_lidar,go2_nav2}-{build,install,log}`
- `/tmp/w05-{build,install,log}`
- `/tmp/w06-{after,before}-{build,install,log}`
- `/tmp/w07-{acceptance-build,after-build,after-install,after-log,before-build,before-install,before-log,before-source,bridge-build,bridge-install,bridge-log,close-build,close-install,close-log,close-test-log,debug-lidar,final-build,final-install,final-log,final-refresh-log,final-test-log}`
- `/tmp/w08-{build,install,log,test-log}`

- 지정 패턴의 `w01*`, `d01*`, `d02*`, `p01*` 디렉터리는 없었다.
- 삭제 크기와 정리 직후 free 증가: 340,959,232 bytes, 약 325.16 MiB.
- 정리 직전 root free: 20,864,315,392 bytes.
- 최종 `df -B1 /` free: 21,205,266,432 bytes, 사용률 64%.
- 최초 build 전 root free: `df -h` 기준 337 MiB, 사용률 100%.
- 중간 약 20 GB 증가는 소유자가 별도로 지시한 다른 프로젝트 삭제에 따른 값이다.
- `~/.ros/log`, `/var/log`, 다른 프로젝트와 `archive/`는 정리 대상에 넣지 않았다.
- 원시 삭제 목록과 감사: `.omo/evidence/f03-durable/scratch-deleted.json`.

## 검증 경계

GUI·실제 Xbox FD/입력·RViz 조작은 소유자 확인 항목이다.
GUI simulator의 누적 double timestamp 장시간 경계는 F02에서 남긴 상태다.
headless 수용 결과는 그 GUI 경로의 장시간 exact pairing이나 실물 성능을 증명하지 않는다.
