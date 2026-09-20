# go2_mujoco_lidar

이 패키지는 공식 `unitree_mujoco`의 Go2 MJCF를 수정하지 않고, simulator가 ROS2로
내보내는 `/lowstate`와 `/sportmodestate`를 별도 MuJoCo model에 반영해 LiDAR ray를
계산할 때 사용한다. 실제 Go2 센서 드라이버를 대체하지 않는 simulation 전용 확장이다.

## 처리 흐름

- `mujoco_lidar_node`가 공식 `scene_terrain.xml`을 읽는다.
- 같은 MuJoCo step에서 채워진 `SportModeState`의 위치·IMU quaternion과
  `LowState.tick`으로 시점이 확인된 12개 관절각을 MJCF의 freejoint와 이름 기반 joint
  주소에 반영한다. 시점 차이가 기본 2 ms를 넘으면 해당 frame을 폐기한다.
- 프로젝트가 수용한 `base -> utlidar_lidar` 좌표에서 `mj_multiRay`를 실행한다.
- raw hit를 `/utlidar/cloud`에 보존하고, 각 점에 `hit_geom_id`와 `hit_body_id`를
  기록한다.
- 같은 simulation-time stamp로 `/clock`, cloud와 `/utlidar/robot_odom`을 발행한다. frame은
  `odom -> base_link`이고, pose·linear velocity는 `SportModeState`, orientation·angular
  velocity는 같은 `SportModeState.imu_state`에서 온다. `LowState`는 관절각에만 사용한다.
- base의 자손 body에 맞은 점만 자기반사로 판정해
  `/simulation/utlidar/cloud_self_filtered`에서 제외한다.
- launch가 filtered cloud를 기존 mapping gate와 공식 `pointcloud_to_laserscan`에
  연결해 `/scan`을 만든다. MuJoCo 다중 수직 레이어가 바닥에 닿아 만드는 동심원형
  투영을 배제하기 위해 simulation launch에서만 converter 최소 높이를 `-0.10 m`로
  올린다. 실물 기본 launch와 공통 YAML의 `-0.25 m` 기본값은 변경하지 않는다.

센서 원점이 전방 하우징 collision 안에 있으므로 raycast에서는 sensor가 속한
`base_link` body 자체를 제외한다. 이는 MuJoCo rangefinder의 동일 body 제외 의미와
같다. 네 다리는 각각 하위 body라 raw cloud에 남고 5단계 필터의 판정 대상이 된다.

## 빌드와 실행

`MUJOCO_ROOT`는 `include/`와 `lib/`가 있는 MuJoCo 배포 경로이고,
`UNITREE_MUJOCO_ROOT`는 공식 저장소 루트다.

```bash
export MUJOCO_ROOT=/path/to/unitree_mujoco/simulate/mujoco
export UNITREE_MUJOCO_ROOT=/path/to/unitree_mujoco
colcon build --packages-select go2_mujoco_lidar
ros2 launch go2_mujoco_lidar go2_mujoco_stage45.launch.py
ros2 launch go2_mujoco_lidar go2_mujoco_stage6_slam.launch.py
```

두 상태 토픽을 모두 받기 전에는 cloud를 발행하지 않는다. 기본 raster는
360×8, 10 Hz, 0.25~5.0 m이며 기존 `/scan`은 0.5° 720-bin 계약을 유지한다.

`go2_mujoco_stage6_slam.launch.py`는 Stage 4/5가 `/scan`을, bringup adapter가
`/odom`을 각각 한 번씩 소유하도록 유지하고, `start_inputs=false`·`use_sim_time=true`로
기존 SLAM Toolbox mapping owner만 추가한다. static TF·scan·odometry를 중복 생성하지 않는다.

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

수평 ray 기본값은 기존 동작을 보존하는 360이다. 0.5° 간격의 720-bin `/scan`과
직접 맞추는 품질 비교에서는 `horizontal_samples:=720`을 명시한다.
수직 layer 기본값도 기존 8개·±15°를 유지한다. 수직 합성 여부를 분리하는 정지
진단에서는 `vertical_samples:=1 vertical_min:=0.0 vertical_max:=0.0`을 함께 명시한다.

`environment_only:=true`는 private LiDAR model의 로봇 geometry를 ray 대상에서만
제외하는 simulation-only A/B 모드다. 원본 MJCF, 시뮬레이터 충돌 모델과 기본값
`false`는 변경하지 않는다. 이 모드는 SLAM 입력 파이프라인과 self-occlusion의 영향을
분리할 때만 사용하며, 실제 Go2에서 다리에 가려지는 영역까지 관측된다는 근거로
사용하지 않는다.

simulator는 ROS overlay 공유 라이브러리 충돌을 피하기 위해 별도의 clean environment에서
설치된 `scene/go2_indoor.xml`로 먼저 실행한다. ROS launch는 simulator를 시작하지 않으며,
upright hold·LiDAR·odometry adapter·SLAM만 합성한다. Runtime QA는 controller가
`/lowcmd`의 단일 publisher인지 확인한다.
