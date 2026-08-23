# go2_local_navigation

## 목적

Unitree Go2와 Jetson AGX Orin에서 센서 계약을 읽기 전용으로 먼저 검증하고,
이후 장애물 정보 보고, mapping, Nav2 costmap, 목적지 주행으로 확장하기 위한
Python 기반 ROS 2 프로젝트다.

현재 상태는 `candidate`(후보)다. static TF와 LiDAR 입력 형식·QoS·주기는 AGX에서
검증했으며, read-only LiDAR acceptance와 odometry source probe도 AGX에서
검증했다. odometry source는 `header=odom`, `child=base_link`로 관찰됐고,
프로젝트는 `base_link → base` mapping과 `go2_state_estimation/odometry_adapter_node`
owner를 수용했다. adapter runtime에서 `/odom`과 `odom → base` dynamic TF를
확인했지만, 이 mapping은 공식 onboard frame 동일성이나 물리 calibration으로
승격하지 않는다.

## 작업 위치와 규칙

이 프로젝트는 저장소의
`remote_agx_home/go2_projects/projects/go2_local_navigation` 경로로 AGX SSHFS
mount되어 있다. 저장소 루트 `AGENTS.md`가 이 경로에 재귀 적용되므로 프로젝트
전용 `AGENTS.md`는 두지 않는다.

프로젝트의 안전 경계와 패키지별 책임은 이 README와 각 패키지 README에서
관리한다. 확인되지 않은 센서 field·QoS·TF·extrinsic·odometry는 계속
`unverified` 또는 `TBD`로 기록한다.

## 현재 기초 패키지

- `go2_sensor`: 센서 topic 계약과 acceptance 경계
- `go2_state_estimation`: odometry source 관찰과 `odom → base` adapter
- `go2_perception`: 장애물·자유공간 보고 경계
- `bringup`: launch와 설정 파일 조합
- `description`: 공식 Go2 전체 URDF와 TF 계약

초기 범위에는 `/cmd_vel`, Unitree Sport API, `/lowcmd`, stand/walk,
navigation service call을 포함하지 않는다.

## TF 설계

```text
map → odom → base
                ├── imu
                ├── radar
                └── utlidar_lidar
```

`map→odom`은 SLAM/localization, `odom→base`는
`go2_state_estimation/odometry_adapter_node`,
센서 frame 연결은 실제 값이 확인된 뒤 해당 구성요소가 소유한다. 현재
canonical model과 프로젝트 root는 모두 `base`이며, 공식 native sensor 의미는
`imu`=차체 IMU, `radar`=기본 Unitree LiDAR, `front_camera`=기본 내장
카메라로 기록한다. runtime `/utlidar/cloud`의 frame_id는
`utlidar_lidar`로 확인했고 static TF 연결도 runtime에서 검증했다. URDF 기준값은
`description/urdf/go2_description.urdf`에서 확인하고, 좌표 확인 이미지는
`description/urdf/go2_description_coordinate_check.png`에서 확인한다.

하나의 TF edge에는 하나의 publisher만 둔다. 센서 입력과 perception 보고는
읽기 전용으로 유지하며, `/cmd_vel`, Unitree control service, motor command,
stand/walk, navigation command는 별도 승인 전까지 추가하거나 실행하지 않는다.

공식 URDF는 구조·기준값을 보관하는 asset이지 실행 승인이나 현재 runtime
TF 검증 결과가 아니다. `map→odom`과 `odom→base`는 URDF fixed
joint로 추가하지 않는다.

## 다음 확인 단계

1. bag/replay acceptance
2. 정지 상태 perception 보고
3. mapping-only와 Nav2 costmap-only 검증
4. 별도 승인 후 motion adapter 검토

센서·odometry source probe 결과는
`records/experiments/go2_local_navigation_sensor_odometry_probe_20260823.md`와
같은 stem의 YAML projection에 보존한다.

adapter mapping 결정과 runtime 결과는 각각
`records/decisions/go2_local_navigation_odom_frame_mapping_20260823.md`,
`records/experiments/go2_local_navigation_odom_adapter_20260823.md`와 같은 stem의
YAML projection에 보존한다.
