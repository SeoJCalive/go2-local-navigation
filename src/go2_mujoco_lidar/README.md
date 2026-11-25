# go2_mujoco_lidar

W04에서 scan·odometry 입력의 소유자는 `go2_inputs`, static TF·DDS 환경 판정의
소유자는 `description`으로 바뀌었다. 아래 공개 sim 명령과 기본값은 유지한다.
runtime은 실물 후보와 `go2_validation`에 의존하지 않는다.
현재 4개 core 구조와 호출 이관은 [W04 기록](../../docs/stabilization-2026-11/W04_BOUNDARIES.md)을 따른다.
반복 가능한 mapping/save/localization/5-goal/reset 수용 시험은
[추적 하네스](test/acceptance/README.md), 기본 시험 분류와 Release 음성 대조는
[W07 inventory](../../docs/stabilization-2026-11/W07_TEST_INVENTORY.md)를 따른다.
빌드·Release 음성 대조·종료 원인과 반복 수용 결과는
[W07 기록](../../docs/stabilization-2026-11/W07_STABILIZATION.md)에 있다.

이 패키지는 공식 `unitree_mujoco`의 Go2 MJCF를 수정하지 않고, simulator가 ROS2로
내보내는 `/lowstate`와 `/sportmodestate`를 별도 MuJoCo model에 반영해 LiDAR ray를
계산할 때 사용한다. 실제 Go2 센서 드라이버를 대체하지 않는 simulation 전용 확장이다.

## 처리 흐름

- `mujoco_lidar_node`가 공식 `scene_terrain.xml`을 읽는다.
- `SportModeState.stamp`와 `LowState.tick × 1 ms`가 정확히 같은 입력만 결합한다.
  최대 128개 미완성 timestamp와 최신 완성 pair 하나를 보관한다. wall timer는
  완성 pair를 한 번만 소비하므로 누락·중복·입력 정지 때 cached 출력을 반복하지 않는다.
- MJCF `imu` site의 base 기준 위치를 offset의 단일 원본으로 사용한다.
  `p_base = p_imu - R r`, `v_base_body = Rᵀ v_imu_world - ω_body × r`로
  한 번 변환한 base pose를 raycast와 odometry 양쪽에 적용한다.
- 프로젝트가 수용한 `base -> utlidar_lidar` 좌표에서 `mj_multiRay`를 실행한다.
- raw hit를 `/utlidar/cloud`에 보존하고, 각 점에 `hit_geom_id`와 `hit_body_id`를
  기록한다.
- 같은 simulation-time stamp로 `/clock`, cloud와 `/utlidar/robot_odom`을 발행한다. frame은
  `odom -> base_link`이고 pose는 base 원점의 world 좌표, twist는 base 원점/축이다.
  orientation·angular velocity는 같은 `SportModeState.imu_state`에서 온다.
  `LowState`는 관절각에만 사용한다. `/clock`은 Sport callback에서 갱신한다.
- 같은 matched pair의 motor q를 Unitree FR·FL·RR·RL hip/thigh/calf 순서로
  `/joint_states`에 발행한다. 다리 TF는 기존 robot_state_publisher가 소유하며
  이 node는 TF를 직접 발행하지 않는다.
- base의 자손 body에 맞은 점만 자기반사로 판정해
  `/simulation/utlidar/cloud_self_filtered`에서 제외한다.
- 기존 ray를 base 좌표계로 회전해 각 base 방위각에서 절대 수직각이 가장 작은 ray
  하나를 초기화 시 선택한다. 선택 ray의 기존 충돌점만
  `/simulation/utlidar/cloud_planar_selected`에 발행하며 miss나 자기반사에는 다른
  layer로 대체하지 않는다. MuJoCo model metadata에서 world body 소속 plane으로
  판정된 floor hit도 planar 출력에서만 miss 처리하며, raw·self-filtered cloud와
  simulator 충돌 geometry는 바꾸지 않는다.
- launch가 planar-selected cloud를 기존 mapping gate와 공식
  `pointcloud_to_laserscan`에 연결해 `/scan`을 만든다. 실물 기본 launch와 공통 YAML은
  변경하지 않는다.

센서 원점이 전방 하우징 collision 안에 있으므로 raycast에서는 sensor가 속한
`base_link` body 자체를 제외한다. 이는 MuJoCo rangefinder의 동일 body 제외 의미와
같다. 네 다리는 각각 하위 body라 raw cloud에 남고 5단계 필터의 판정 대상이 된다.

## 빌드와 실행

`MUJOCO_ROOT`는 `include/`와 `lib/`가 있는 MuJoCo 배포 경로이고,
`UNITREE_MUJOCO_ROOT`는 공식 저장소 루트다.

새 환경에서는 [고정 simulator·MuJoCo 설치 절차](../../third_party/unitree_mujoco/README.md)로
필수 상태 패치를 적용한 뒤 두 환경변수를 지정한다.

```bash
export MUJOCO_ROOT=/path/to/unitree_mujoco/simulate/mujoco
export UNITREE_MUJOCO_ROOT=/path/to/unitree_mujoco
colcon build --packages-select go2_mujoco_lidar
ros2 launch go2_mujoco_lidar go2_mujoco_stage45.launch.py
ros2 launch go2_mujoco_lidar go2_mujoco_stage6_slam.launch.py
```

두 상태 토픽을 모두 받기 전에는 cloud를 발행하지 않는다. 기본 raster는
720×8, 수직 ±15°, 10 Hz, 0.25~5.0 m다. 모든 sim launch는 같은 기본값을 쓰며
planar-selected cloud에서 `/scan`의 0.5° 720-bin 계약을 유지한다.

입력 시간이 역행하면 pending pair와 소비 이력을 비우고 두 stream의 새 epoch를
기다린다. reset마다 경고 한 번을 남긴다. 1초 동안 새 matched step이 없으면
steady clock 기준 경고 한 번을 남기고, 재수신 시 복구를 기록한다.
use_sim_time인 mapping gate와 odometry adapter도 공유 clock-jump helper로 baseline을
초기화하므로 node 재시작 없이 cloud와 odom이 복구된다. SLAM/AMCL 지도 reset과
이전 지도에서 navigation을 재개하는 정책은 이 입력 복구 계약에 포함하지 않는다.

`go2_mujoco_stage6_slam.launch.py`는 Stage 4/5가 `/scan`을, go2_inputs adapter가
`/odom`을 각각 한 번씩 소유하도록 유지하고, `start_inputs=false`·`use_sim_time=true`로
기존 SLAM Toolbox mapping owner만 추가한다. static TF·scan·odometry를 중복 생성하지 않는다.

## 통합 simulation 입력 실행

`go2_sim_bringup.launch.py`는 이미 clean environment에서 실행한 physics simulator의
PID를 받아 LiDAR·cloud gate·scan·static TF·odometry adapter와 선택적 SLAM을
한 번에 시작한다. simulator와 RL controller는 기존 수동 절차대로 별도로 실행한다.
이 launch는 이 두 외부 process를 종료하거나 재시작하지 않는다.

```bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ROS/Unitree 및 선택한 local_navigation install을 source한 셸에서 실행한다.
# SIM_PID는 사용자가 시작한 unitree_mujoco -s <MJCF> -i 1 -n lo의 실제 PID다.
ros2 launch go2_mujoco_lidar go2_sim_bringup.launch.py \
  simulator_pid:="$SIM_PID" mapping:=true
```

- world의 단일 원본은 `/proc/$SIM_PID/cmdline`의 `-s`다. 상대 경로는 simulator
  cwd에서 해석하며 raycast에도 그 resolved path를 전달한다. 선택적
  `model_path:=<expected MJCF>`가 다른 파일을 가리키면 node 시작 전에 거부한다.
- 시작 로그에 world resolved path와 SHA-256을 기록한다. 이 hash는 현재 MJCF
  파일의 provenance이며 이미 로드된 simulator 메모리의 checksum은 아니다.
  simulator를 실행하는 동안 MJCF와 include/asset을 수정하지 않는다.
- ROS와 physics의 domain이 같아야 하고 physics는 `-n lo`여야 한다.
  ROS Domain 0 또는 loopback 격리가 없는 실행은 기본 거부한다.
  기존 runbook의 Domain 1·`ROS_LOCALHOST_ONLY=0`도 실제 `CYCLONEDDS_URI` 파일이
  CycloneDDS lo-only 설정이면 허용한다. 단순히 파일 이름으로 판정하지 않는다.
- `allow_unsafe_simulation:=true`는 domain-zero/network 경계의 명시적 override다.
  world와 physics/ROS domain 불일치는 override하지 않는다.
- 모든 ROS node는 `use_sim_time=true`다. `/clock`은 LiDAR, `odom -> base`는
  adapter, `base -> utlidar_lidar`는 static publisher, URDF fixed TF는
  robot_state_publisher, `map -> odom`은 `mapping:=true`일 때 SLAM만 소유한다.
- 하나의 ROS process가 종료하면 launch가 나머지 소유 process도 종료한다.
  `raycast_error_limit`(기본 3)은 연속 실패 수다. 각 오류를 기록하고 성공한 scan은
  계수를 초기화한다. 예산 소진은 LiDAR exit 1로 이어져 전체 launch를 종료한다.
- mapping 중 physics reset 뒤에는 이 launch 전체를 재시작한다. W02의 입력 epoch
  복구가 SLAM 지도 epoch 복구를 보장하지 않는다.

기존 stage45·stage6·stage62 진입점도 raster 기본값 `720×8`·수직 ±15°를 쓴다.
stage45와 이를 포함하는 launch에는 같은 domain/loopback guard와
`allow_unsafe_simulation`·`raycast_error_limit` argument가 추가됐다.
통합 진입점을 쓰면 기존 runbook의 ROS 터미널 3(LiDAR/scan), 4(odom), 5(SLAM)를
위 명령 하나로 대체한다. 이때 세 launch를 별도로 중복 실행하지 않는다.
simulator/RL·RViz·지도 저장 절차와 bridge 저장소는 바꾸지 않았다.

## Stage 6-2 indoor upright hold

`go2_mujoco_stage62_upright_hold.launch.py`는 simulation-only Domain 1에서
`/lowcmd`의 단일 owner인 `go2_mujoco_upright_hold`와 indoor LiDAR model을 함께
기동한다. controller는 공식 `stand_go2.cpp`의 12개 stand target, torque mode,
CRC, `tanh(t / 1.2)` ramp를 사용한 뒤 stand pose를 유지한다. gait, velocity,
translation command는 만들지 않는다.

launch는 `ROS_DOMAIN_ID=1`과 package-owned loopback CycloneDDS file URI를
명시한다. controller도 시작 시 같은 조건과 `lo` 또는 `127.0.0.1`이 포함된
CycloneDDS configuration을 다시 검사하므로, Domain 0이나 비-loopback 설정에서는
`/lowcmd` publisher를 만들지 않고 종료한다. console에는 `/lowcmd` owner node가
기록되어 command ownership을 확인할 수 있다.

`scene/go2_indoor.xml.in`은 official `go2.xml`을 include하고 6×6 m 외벽, 내부 L wall,
box pillar를 명시 geometry로 추가한다. MuJoCo는 MJCF include에서 environment
variable을 확장하지 않으므로 CMake가 `UNITREE_MUJOCO_ROOT`의 official model
절대 경로를 설치된 `scene/go2_indoor.xml`에 기록한다.

```bash
export MUJOCO_ROOT=/path/to/unitree_mujoco/simulate/mujoco
export UNITREE_MUJOCO_ROOT=/path/to/unitree_mujoco
colcon build --packages-select go2_mujoco_lidar
source install/setup.bash
ros2 launch go2_mujoco_lidar go2_mujoco_stage62_slam.launch.py
```

수평 ray 720개, 수직 layer 8개·±15°가 sim 기본값이다. 이는 9/28 good-map의
sensor-frame raster이며, base-frame에서 수직각이 가장 작은 ray를 방위별로 고른다.
sensor-frame 단일 layer와 같은 geometry가 아니다.

`environment_only:=true`는 private LiDAR model의 로봇 geometry를 ray 대상에서만
제외하는 simulation-only A/B 모드다. 원본 MJCF, 시뮬레이터 충돌 모델과 기본값
`false`는 변경하지 않는다. 이 모드는 SLAM 입력 파이프라인과 self-occlusion의 영향을
분리할 때만 사용하며, 실제 Go2에서 다리에 가려지는 영역까지 관측된다는 근거로
사용하지 않는다.

simulator는 ROS overlay 공유 라이브러리 충돌을 피하기 위해 별도의 clean environment에서
설치된 `scene/go2_indoor.xml`로 먼저 실행한다. ROS launch는 simulator를 시작하지 않으며,
upright hold·LiDAR·odometry adapter·SLAM만 합성한다. Runtime QA는 controller가
`/lowcmd`의 단일 publisher인지 확인한다.
