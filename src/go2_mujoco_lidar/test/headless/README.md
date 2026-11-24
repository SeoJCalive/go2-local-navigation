# W02 headless 상태 통합 시험

MuJoCo의 실제 qpos와 DDS를 거친 cloud/odom을 비교할 때 사용한다.
GUI용 main 대신 작은 driver에서 W01에 고정된 `Go2Bridge::run()`과 `mj_step()`을
호출한다. simulator checkout과 header patch는 [W01 pin](../../../../third_party/unitree_mujoco/README.md)을
따른다. simulator source를 수정하지 않는다.

중력을 끈 freejoint를 yaw ±90°로 놓고 첫 1초 정지 후 world ±Y 속도 0.2 m/s를
설정한다. 각 6초 epoch 사이 `mj_resetData()`로 시간이 역행한다.
이는 실제 MuJoCo 센서·DDS·좌표·입력 복구 시험이며 RL/joystick 정책 주행 시험은 아니다.
모든 publisher/subscriber는 Domain 1과 localhost에 격리한다. 전용 Discovery Tag로
이미 실행 중인 Domain 1 세션도 분리한다. 물리 명령은 발행하지 않는다.
ROS의 loopback 선택은 `ROS_LOCALHOST_ONLY=1`이 맡고 SDK에는 driver가 `lo`를
명시한다. Humble RMW가 interface 설정을 앞에 추가하므로 공통 XML에 `lo`를 다시
나열하면 같은 interface 중복 선택으로 실패한다.

AGX에서 W01의 `UNITREE_MUJOCO_ROOT`와 `MUJOCO_ROOT`, ROS 및 새 install overlay를
지정한 셸에서 실행한다. SDK CMake prefix는 기존 AGX `/opt/unitree_robotics`다.

```bash
cmake -S src/go2_mujoco_lidar/test/headless -B /tmp/w02-headless-build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/w02-headless-build -j 4
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/go2_mujoco_lidar/test/headless/cyclonedds_w02.xml"
python3 src/go2_mujoco_lidar/test/headless/verify_runtime.py \
  --simulator /tmp/w02-headless-build/w02_headless \
  --lidar /tmp/w02-install/go2_mujoco_lidar/lib/go2_mujoco_lidar/mujoco_lidar_node \
  --model "$UNITREE_MUJOCO_ROOT/unitree_robots/go2/scene_terrain.xml" \
  --dds-config "$PWD/src/go2_mujoco_lidar/test/headless/cyclonedds_w02.xml" \
  --output "$PWD/.omo/evidence/w02-state-runtime"
```

`truth.csv`는 독립 base qpos, `observations.json`은 ROS 수신값, `result.json`은
epoch별 수신 수·중복 수·오차·정지 뒤 출력 수를 남긴다. 각 node 로그와 종료한
owned PID 목록도 보존한다. `mj_step()`은 qpos 적분 전 센서를 계산하므로 이동
중 센서가 qpos보다 1 step 뒤일 수 있다. 2 ms × 0.2 m/s = 0.4 mm와 DDS float32
양자화를 포함해 1 mm 이내를 검사한다. 이 오차 기준은 navigation 정확도 기준이 아니다.

## 2026-11-24 AGX 결과

- simulator checkout HEAD는 W01 pin `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`였다.
- 사용한 bridge header SHA-256은 W01 patch 적용본과 같은
  `fbcae26bc1b59a93023b6c8fc157ca2de8e93e2adb60bd84e9bac27ec00a1b51`이었다.
- `/tmp/w02-build`·`/tmp/w02-install`에 의존 패키지를 포함한 8개 새 Release build가
  성공했다. 최종 변경 LiDAR build도 exit 0이었다.
- state estimation Python 23개, perception Python 14개가 통과했다.
- LiDAR CTest 5/5가 통과했다. 내부 Python 12개와 C++ executable 2개이며,
  신규 state executable은 NDEBUG에서도 독립 기하·pairing·reset 검사 46개를 실행했다.
- colcon 집계는 CTest의 Python wrapper도 포함해 54 tests, 0 errors, 0 failures,
  0 skipped였다. 기존 planar executable의 Release assert 문제(F20)는 별도 미수정이다.
- headless integration exit 0이었다. 위치 표본 121개의 base qpos 대비 최대 오차는
  `0.0004000296754269428 m`였다.
- yaw ±90°에서 body-forward 표본 99개의 body x 속도는
  `0.1994348060~0.1996703199 m/s`, 최대 절대 body y 속도는 `0.000001896993 m/s`였다.
- raw cloud·source odom·validated cloud·canonical odom은 reset 전후 각각 60/61개였다.
- 네 stream의 동일 epoch/stamp 중복과 raw cloud/source odom stamp 불일치는 0이었다.
- LiDAR·cloud gate·odom adapter는 같은 PID로 유지됐고 clock 역행을 각각 한 번 기록했다.
- simulator 종료 0.5초 뒤부터 2초 관찰 종료까지 네 stream의 출력은 0이었다.
- 입력 정지 경고는 초기 입력 대기와 마지막 정지에 각각 한 번 기록됐다.
- 시작한 child PID 4개는 모두 wait/reap 뒤 `/proc`에서 사라졌다.
- 기존 Domain 1 simulator와 LiDAR는 건드리지 않았고 전용 Discovery Tag로 분리했다.
- 원시 결과·truth·관찰·로그는 `.omo/evidence/w02-state-runtime/`에 보존했다.

이 결과는 별도 headless driver의 무중력 자유 운동과 실제 pinned DDS bridge에 대한
관찰이다. GUI simulator main의 동시 실행 scheduling, RL/joystick 주행, SLAM/AMCL
지도 reset·재사용과 실제 로봇 동작은 검증하지 않았다.
