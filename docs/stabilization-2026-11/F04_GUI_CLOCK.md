# F04 공유 GUI simulator의 정수 epoch clock

소유자 GUI 시험 전에 공유 simulator의 장시간 exact state pairing을 확인하는
수정과 재현 근거다. 실행 명령은 [GUI clock 검증](../../src/go2_mujoco_lidar/test/acceptance/README.md#gui-binary-장시간-clock-검증)을 따른다.

## 원인과 변경

W01 header의 `RobotBridge::run()`은 Low tick을 `round(time/.001)`,
Sport stamp를 `llround(time*1e9)`로 만들었다. GUI `main.cc`의 두 `mj_step()`은
double time을 누적한다. 별도 recurrent bridge thread는 physics/UI mutex 없이
그 시간을 두 번 읽으므로 서로 다른 step을 읽을 수도 있었다.
LiDAR는 `Sport.stamp == Low.tick × 1 ms`만 결합한다.

`0003-gui-integer-epoch.patch`는 pinned upstream과 0001 위에 적용한다.
GUI `main.cc:467,508`의 적분은 `simulation_step.h`의 epoch counter를 사용하고,
reset/history로 시간이 바뀌면 다음 step 전에 counter를 재설정한다.
`main.cc:625`는 physics/UI mutex 안에서 전체 `interface->run()`을 호출한다.
Backspace reset도 같은 mutex를 사용한다.
`unitree_sdk2_bridge.h:182,232,237`은 한 snapshot의 nanosecond 값에서 두 stamp를 만든다.
기존 acceptance helper는 이 upstream patch의 helper를 include해 중복을 제거했다.
정확한 pairing 허용오차·Nav2 설정·물리 적분기는 바꾸지 않았다.

## 독립 재현과 빌드

- pin: `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`.
- shared 수정 전 config/header 백업:
  `/home/bi-agx1/go2_projects/archive/unitree_mujoco_backup_20261125/`.
- 백업 header SHA-256:
  `fbcae26bc1b59a93023b6c8fc157ca2de8e93e2adb60bd84e9bac27ec00a1b51`.
- 백업 config SHA-256:
  `c12c37790be68fc7f8fc6c0200c192e21c67e019a3fe59aa41c3f58ebd25e9b1`.
- setup은 shared와 기본 영구 root 각각에서 pinned `git archive` 임시 tree에
  0001·0003 및 선택한 0002를 clean apply하고 네 파일의 byte 비교를 통과했다.
- shared는 0002 config와 기존 evidence를 보존한다. 기존 SDK build의 root 소유
  `install_manifest.txt`는 보존하고 `shared/dependencies/sdk-build`를 별도로 사용했다.
- shared와 영구 기본 root의 GUI·acceptance Release build 및 clock CTest는 exit 0이었다.
- 최종 shared/default의 main·bridge header·step helper는 각각 `cmp` exit 0이었다.
- 최종 main SHA-256:
  `66e7d4845b06258051367516855705a9362307ac2edc64121b68eb1a839cd3b8`.
- 최종 bridge header SHA-256:
  `489fc216ae1bffe544413003abb118718175230632a5aaf0a11f63b1924b018c`.
- 최종 step helper SHA-256:
  `e840047359e97b91c2b1f38fce2a3ef95935b0a92b4ce924a0d3e9a1b7eee5bc`.
- 최종 shared GUI binary SHA-256:
  `8cd621b1d848198402c3dc9d7620945335d7eb0c4bd9da0254cae417e0f4b708`.
- 실제 MuJoCo 빈 model에서 기존 누적 산술의 첫 Low/Sport 불일치는
  step `128601`, sim time `257.202 s`였다.
- 정수 2 ms oracle와 새 helper는 두 epoch 각각 600초를 통과했다.
  두 번째 reset은 counter를 외부에서 0으로 바꾸지 않아 GUI reset 재설정도 검사했다.
- shell/Python 문법과 patch 외 파일의 `git diff --check`는 통과했다.
- 0003의 upstream context 공백 15행은 byte 재현을 위해 보존했다.
  patch 파일 자체의 `git diff --check`는 이 15행을 trailing whitespace로 진단한다.

공유 수정은 setup으로만 적용했다. 테스트 전용 config는 공유 binary의 동일 inode
hard link 옆에 pinned 기본 config를 복사해 격리한다. 공유 config와 기존 FIFO를
변경하거나 제거하지 않는다.

## 실제 GUI binary의 긴 실행과 reset

`runs/f04-gui-clock-04/`는 Xvfb `:1`·Mesa llvmpipe에서 shared GUI binary와
설치된 sensor bringup을 실행한 결과다. binary는 shared 원본과 동일 inode이며,
별도 acceptance main으로 대체하지 않았다. `gui.png`에서 실제 UI·world·robot
render를 직접 확인했다. Domain 41·`lo`·multicast off·loopback peer를 사용했다.
ROS의 `ROS_LOCALHOST_ONLY=0`은 interface 중복 주입을 피하며 실제 XML은 lo-only다.

| 관찰 | reset 전 | reset 후 |
| --- | --- | --- |
| Sport 수신 수 | 533,674 | 4,363 |
| Low 수신 수 | 518,298 | 4,260 |
| exact Low/Sport 공통 step | 291,589 | 2,442 |
| cloud·odom 각각 | 6,086 | 51 |
| cloud·odom stamp 범위 s | 2.290..610.776 | 0.082..5.086 |
| cloud·odom 최대 간격 s | 0.126 | 0.118 |

- Sport stamp의 정수 ms 격자 불일치는 전체 0이었다.
- cloud·odom의 reset 전 연속 구간은 각각 608.486초였다.
- 원시 `stamps.json` 집계에서 257초 이후 cloud·odom은 각각 3,538개였다.
- XTest로 `MuJoCo : go2_nav2_complex_indoor` 창에 Backspace press/release를 보냈다.
- 네 stream은 모두 epoch 역행을 정확히 한 번 관찰했고 같은 process로 새 입력을 받았다.
- wall duration은 620.607350초였고 전체 판정은 `passed: true`, command exit 0이었다.
- 세 owned session은 SIGINT만 사용했으며 최장 stop은 0.685705초, 잔류는 0이었다.
- 공유 config·기존 FIFO는 보존했고 ROS/RL motion command는 발행하지 않았다.

## 같은 변경 뒤 기존 수용 시험 한 번

GUI 정리가 끝난 뒤 영구 기본 경로로 기존 `acceptance.py`를 한 번 실행했다.
원시는 `runs/f04-acceptance-01/`이며 Domain 41·전용 discovery tag·lo 격리를 사용했다.

| 판정 | 결과 |
| --- | --- |
| mapping | 13/13점, 234.342555초, stall 0 |
| 지도 저장 | 320×239, 0.05 m/cell, YAML·PGM·session sidecar |
| G1 | SUCCEEDED, 독립 qpos XY 오차 0.203262 m |
| G2 | SUCCEEDED, 독립 qpos XY 오차 0.135106 m |
| G3 | SUCCEEDED, 독립 qpos XY 오차 0.087961 m |
| G4 | SUCCEEDED, 독립 qpos XY 오차 0.079519 m |
| G5 | SUCCEEDED, 독립 qpos XY 오차 0.175323 m |
| reset | cloud/odom 각각 3076→3080, stamp 326.574→0.134초, 같은 PID |
| 접촉 | mapping 및 전체 비바닥 robot/world 접촉 0 |
| 종료 | 여섯 lifecycle FINALIZED=4, TERM/KILL escalation 0, owned 잔류 0 |
| 종합 | 12개 criteria 모두 true, `passed: true`, command exit 0 |

기록된 F04 실행 전체의 PID 46개·session 16개의 합집합을 `/proc`에서 별도 감사했다.
잔류는 0이었고 acceptance FIFO도 제거됐다. GUI와 acceptance의 기존 process
snapshot은 모두 보존됐다. reset 때 이전 epoch TF 경고는 있었으며 입력 복구와
종료 판정은 통과했다.

## 검증 경계

실제 Xbox 입력·물리 display/GPU·RViz 조작은 소유자 GUI 시험으로 남는다.
다른 world·timestep·policy, 지도 전체 정확도, reset 뒤 AMCL 재초기화와 goal 재개,
실제 로봇 동작은 이 결과의 검증 범위 밖이다. 전체 colcon suite는 다시 실행하지 않았다.
