<!-- 목적: simulation runtime overlay 준비·검증 절차. 입력 pin/parameter를 build와
실행 증거로 연결하며 시스템·실물 설정과 구분한다. 실물 성능은 미검증이다. -->
# Simulation SLAM Toolbox overlay

24회 offline 동등성은 통과했으나 실제 G02 attempt 9는 pose drift와 이중벽으로
실패했다. 정상 fine candidate의 다수 거부도 확인했다. simulation 지도 완료 판정은
[runtime 검증 기록](../../docs/stabilization-2026-11/G02_RUNTIME_GATE.md)을 따른다.

simulation mapping에서 검증된 거리 gate를 사용할 때 준비하는 source overlay다.
원본은 https://github.com/SteveMacenski/slam_toolbox.git 의 `2.6.10`,
commit `8293d21fe5c816d0405e0cc1f3eb46c70659c267`이다.
시스템 `/opt/ros/humble`과 실물 `slam_mapping.yaml`은 그대로 둔다.

ROS Humble과 Unitree overlay를 source한 프로젝트 루트에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/bi-agx1/unitree_ros2/cyclonedds_ws/install/setup.bash
bash third_party/slam_toolbox/setup.sh
source third_party/slam_toolbox/install/local_setup.bash
source install/local_setup.bash
ros2 pkg prefix slam_toolbox
```

마지막 출력은 프로젝트 `third_party/slam_toolbox/install/slam_toolbox`여야 한다.
정규 simulation launch는 ament에서 이 overlay가 선택됐는지 확인한다.
`install/setup.bash`로 과거 underlay를 다시 source하면 순서가 바뀔 수 있으므로
위의 `local_setup.bash` 순서를 사용한다. 선택 작업 root와 local 원본 git은
`setup.sh /absolute/root /absolute/source-repository`로 지정한다.
재현 가능한 source·build·install·log는 이 디렉터리 아래 Git 제외 경로에 둔다.
version·source commit·실제 SDK SHA256은 `install/go2-slam-runtime.json`에 남긴다.

`loop_closure_max_odom_distance`는 double·미터이며 기본 `0.0`은 비활성이다.
simulation 진입점만 `1.0`을 넘긴다. fine matcher가 기존 수락 조건을 통과하면
proposed sensor pose의 barycenter로 실제 linked anchor를 구한다. 두 node의
원래 base odometry XY 직선거리가 상한보다 클 때만 pose 변경·edge·최적화
전에 거부한다. 경로 누적 거리나 corrected pose를 사용하지 않는다.
매 거부는 `LOOP_ODOM_GATE_REJECT`에 node ID·거리·상한을 기록한다.
필드는 archive에 추가하지 않아 기존 posegraph 직렬화 순서를 유지한다.

기존 `karto-observation.patch`는 `G02_*_EVIDENCE` 경로를 지정한 검증 실행에서만
raw scan·candidate·실제 edge를 저장한다. runtime patch는 관측 stdout도 환경
변수로 제한한다. 비활성 gate는 추가 anchor 계산을 건너뛰며 수락·지도 동작을
upstream 그대로 유지한다. 이 overlay는 다른 world·실물 odometry drift를
검증하지 않으며 실물 설정에는 gate를 활성화하지 않는다.

## Built runtime 동등성 재생

기존 counterfactual worker1 solver와 입력을 유지하면서 설치된 runtime
`libkartoSlamToolbox.so`와 `libtoolbox_common.so`를 직접 연결한다.
SDK 내부의 runtime gate만 수락을 결정한다. replay callback gate는 호출하지 않는다.
`go2RuntimeGateVersion` 강한 symbol과 RPATH로 시스템 SDK의 혼입을 차단한다.

```bash
H=src/go2_mujoco_lidar/test/wall_hug
E="$PWD/.omo/evidence/loop-gate-runtime"
cmake -S "$H/counterfactual" -B "$E/adapter" \
  -DENGINE_SOURCE="$PWD/.omo/evidence/loop-gate-replay/engine-motion-v2/source" \
  -DRUNTIME_SOURCE="$PWD/third_party/slam_toolbox/build/source" \
  -DRUNTIME_PREFIX="$PWD/third_party/slam_toolbox/install/slam_toolbox"
cmake --build "$E/adapter" --target engine_replay -j2
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 "$H/runtime_equivalence.py" \
  --engine "$E/adapter/engine_replay" --output "$E/equivalence" --jobs 3
```

12 dataset의 parameter `0.0`과 `1.0` 각각에서 기준 edge·불변 factor·online/
final pose·PGM byte 동일성과 지도/pose 지표 오차 `1e-8` 이하를 요구한다.
parameter `1.0`은 엄격한 오류 0도 요구한다. 결정론적 회귀는 정식
`wall_hug_runtime` CTest에 등록했다. native 자산이 없는 CI는 그 검사만 skip한다.
실제 GUI는 ROS source 순서를 지킨 뒤
`python3 -u "$H/runtime_gui.py" --g02`로 기존 route를 실행한다.
기존 physics/GUI helper를 쓰는 AGX 전용이며 `live/progress.log`와 capture는
`.omo/evidence/g02-wall-hug/live/`에 게시한다. 지도·접촉 기준은 그대로다.
