# MuJoCo 시뮬레이터 의존성 설치

새 Linux x86_64/aarch64 환경에서 이 프로젝트의 simulator와 LiDAR 빌드 의존성을
준비할 때 사용한다. 외부 공유 checkout을 수정하는 대신 이 프로젝트가 요구하는
patch와 설치 절차를 `third_party/unitree_mujoco/`에서 함께 관리한다.

## 고정 원본과 패치

- simulator 원본: https://github.com/unitreerobotics/unitree_mujoco.git
- simulator commit: `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`.
- SDK 원본: https://github.com/unitreerobotics/unitree_sdk2.git
- SDK commit: `9754cd153af3da471b0fe5f3aa535e426fb11db3`.
- MuJoCo: `3.3.6`; 공식 Linux release archive의 아키텍처별 SHA-256은 `setup.sh`에 고정했다.
- `0001-sport-state-simulation-time-imu.patch`: SportModeState의 simulation timestamp,
  IMU quaternion/gyro와 yaw_speed를 채운다. LiDAR의 clock·state 입력에 필수다.
- `0002-agx-config-snapshot.patch`: 2026-11-24 AGX의 나머지 미커밋 변경을 보존한다.
  형제 bridge world 절대 경로와 joystick FIFO가 들어 있으므로 자동 설치에는 적용하지 않는다.
  해당 world/FIFO의 생성·실행은 별도 세션 설정이다.

2026-11-24 AGX에서 `ldd simulate/build/unitree_mujoco`가 연결하는 실제
`libmujoco.so.3.3.6`을 Python `ctypes.CDLL`로 로드했다.
`mj_versionString()`은 `3.3.6`, `mj_version()`은 `336`이었다.
`simulate/mujoco`의 실제 경로는 `shared/dependencies/mujoco-dist/mujoco-3.3.6`이었다.
SDK checkout은 tracked clean이었고, 설치된 `/opt/unitree_robotics/lib/libunitree_sdk2.a`와
해당 commit의 `lib/aarch64/libunitree_sdk2.a`는 SHA-256
`a084cc0087b6dc1b6361f874aabed8bd525f0574b3b2228f91b4b512e1bf035e`로 같았다.

AGX simulator CMake cache는 `Unix Makefiles`, `Release`, `/usr/bin/c++`(GCC 11),
`unitree_sdk2_DIR=/opt/unitree_robotics/lib/cmake/unitree_sdk2`를 사용한다.
설치 스크립트도 같은 simulator target과 Release/Makefiles 설정을 사용하되,
SDK는 전용 prefix에 설치한다. SDK example은 설치에 필요하지 않아 빌드하지 않는다.

## 설치와 프로젝트 빌드

Ubuntu 22.04 기준 시스템 의존성을 먼저 설치한다.

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  git ca-certificates curl cmake make g++ \
  libyaml-cpp-dev libspdlog-dev libboost-program-options-dev \
  libglfw3-dev libeigen3-dev libfmt-dev
```

ROS overlay를 source하지 않은 셸에서 프로젝트 루트를 기준으로 실행한다.
`SIM_DEPS`는 기존 공유 dependency와 겹치지 않는 전용 절대 경로로 정한다.
스크립트는 재실행할 수 있고, pin 또는 tracked 변경이 다르면 덮어쓰지 않고 실패한다.
기본 root는 `$HOME/.local/share/go2-local-navigation-sim`이다.
archive 해시는 매번 검증하며, 성공 시 simulator와 `jstest`를 빌드한다.

```bash
SIM_DEPS="$HOME/.local/share/go2-local-navigation-sim"
bash third_party/unitree_mujoco/setup.sh "$SIM_DEPS"
export UNITREE_MUJOCO_ROOT="$SIM_DEPS/unitree_mujoco"
export MUJOCO_ROOT="$SIM_DEPS/mujoco-3.3.6"

source /opt/ros/humble/setup.bash
# AGX의 기존 Unitree ROS 메시지 overlay. 새 장비에서는 CI의 pinned 메시지 checkout 사용.
source /home/bi-agx1/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --symlink-install --base-paths src --event-handlers console_direct+
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

CI도 같은 스크립트를 사용하고 두 root 환경변수를 이후 build/test step에 전달한다.
기존 MuJoCo C++ 테스트는 GUI나 ROS publisher 없이 실행한다.
이 설치 절차는 simulator·ROS node·로봇 동작을 시작하지 않는다.

수용 시험에는 GUI binary 대신 qpos·접촉 기록·reset·전용 DDS discovery를 제공하는
추적 native driver가 필요하다. 형제 RL 저장소의 기존 object·config·policy를 사용하는
아래 선택 빌드로 준비한다. simulator와 재링크 controller는 모두 영구 root에 남는다.

```bash
# ROS overlay를 source하지 않은 clean shell
BUILD_ACCEPTANCE=1 bash third_party/unitree_mujoco/setup.sh
# RL_ROOT를 지정하면 다른 기존 RL checkout/object를 선택할 수 있다.
# 출력: $SIM_DEPS/acceptance-build/acceptance_simulator
#       $SIM_DEPS/acceptance-build/rlroot/deploy/robots/go2/build/go2_ctrl
```

기본 RL 원본은 프로젝트의 형제 `go2_rl_locomotion_baseline`이다.
`rlroot`의 config/logs는 이 저장소를 참조하고 controller의 FSM/policy object는 바꾸지 않는다.
빌드 직전마다 `df`를 출력한다. default workspace의 `BUILD_TESTING=ON` 빌드는
`install/go2_mujoco_lidar/lib/go2_mujoco_lidar/tf_owner_observer`도 설치한다.

## AGX 변경분의 독립 재현 확인

공유 checkout에는 쓰지 않고 `/tmp`의 전용 설치 루트에서 확인한다.
`setup.sh`로 0001을 적용한 뒤 아래 비교를 수행한다.

```bash
diff -u "$SHARED_SIM/simulate/src/unitree_sdk2_bridge.h" \
  "$SIM_DEPS/unitree_mujoco/simulate/src/unitree_sdk2_bridge.h"
git -C "$SIM_DEPS/unitree_mujoco" apply \
  "$PWD/third_party/unitree_mujoco/0002-agx-config-snapshot.patch"
diff -u "$SHARED_SIM/simulate/config.yaml" "$SIM_DEPS/unitree_mujoco/simulate/config.yaml"
```

두 diff의 exit 0은 두 modified 파일의 byte 일치를 뜻한다. 0002 적용 후에는
스크립트의 tracked 변경 검사에 걸리므로 재실행 멱등성은 0002 적용 전에 확인한다.
검증 디렉터리는 확인 후 제거한다. 현재 공유 simulator binary가 이 source에서
만들어졌다는 증명이나 실제 DDS timestamp·quaternion 관찰은 별도 runtime 검증이다.

## 2026-11-24 AGX 검증 결과

- `/tmp/w01_verify`에서 GitHub의 위 고정 commit 두 개를 새로 fetch/checkout했다.
- 새로 받은 MuJoCo aarch64 archive의 SHA-256은
  `dd6389a008a1a4e4678b4a6118c0a099621f3f26721b6f05bb689a92250850b5`로 공식 digest와 같았다.
- 0001 적용 header는 공유 수정본과 `diff -u` 출력 없음, exit 0이었다.
  두 파일 SHA-256은 `fbcae26bc1b59a93023b6c8fc157ca2de8e93e2adb60bd84e9bac27ec00a1b51`이었다.
- 0002 적용 config도 공유 수정본과 `diff -u` 출력 없음, exit 0이었다.
  두 파일 SHA-256은 `c12c37790be68fc7f8fc6c0200c192e21c67e019a3fe59aa41c3f58ebd25e9b1`이었다.
- 전용 SDK prefix로 simulator Release 빌드가 exit 0이었다.
- `setup.sh` 두 번째 실행도 exit 0이었고 header·기본 config·simulator binary의 SHA-256이
  실행 전후 같았다. binary SHA-256은
  `3a0ca247be171c11a94692b07a8b98e1ea40baf5ba88ef3c577d52ea97d35de4`였다.
- 기존 AGX ROS/Unitree 메시지 overlay를 사용한 9개 패키지의 새 Debug build가 exit 0이었다.
  build/install/log는 모두 `/tmp/w01_verify/ws-*`를 사용했다.
- ROS node를 생성하지 않는 기존 Python 계약 테스트 15개가 통과했다.
- 새 Debug build의 `ctest -R '^planar_selection_test$' --output-on-failure`가 1/1 통과했다.
- CI YAML parsing, 5개 run 블록과 `setup.sh`의 `bash -n`이 통과했다.
- patch 파일의 빈 context 행·upstream 원래 행 끝 공백은 byte 재현을 위해 보존했다.
  따라서 patch 파일 자체의 `git diff --check`에는 공백 진단이 있다.
  snapshot reverse apply에도 upstream 원래 공백 복원 경고 1개가 있었으며 변경하지 않았다.
- GitHub hosted x86_64 job과 실제 DDS 상태·기존 simulator binary의 source identity는 미검증이다.
  GUI·ROS node·로봇 동작은 실행하지 않았다.
