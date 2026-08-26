# Launch 경계

## `go2_sensor_acceptance.launch.py`

이 launch는 static TF를 추가로 시작하지 않는다. 이미 실행 중인 static TF와
동시에 사용하며, `go2_sensor`의 LiDAR acceptance node와
`go2_state_estimation`의 odometry source probe만 시작한다.

- `/utlidar/cloud` read-only subscription
- `/utlidar/robot_odom` read-only subscription
- `/tf`, `/tf_static`, `/odom`, `/cmd_vel` publish 없음

2026-08-23 AGX에서 LiDAR valid `395`·invalid `0`, odometry received `3851`·invalid
`0`을 확인했다. `base_link`와 `base`의 동등성 및 `odom → base` owner는 이 launch에서
결정하지 않는다.

## `go2_static_tf.launch.py`

이 launch는 설치된 `description` 패키지에서 `urdf/go2_description.urdf`를 찾고,
다음 구성요소만 시작한다.

- `robot_state_publisher` 1개: canonical URDF의 모델 TF를 publish한다.
- `tf2_ros static_transform_publisher` 1개: parent `base`, child
  `utlidar_lidar`, x=`0.28945`, y=`0.0`, z=`-0.046825`, roll=`0.0`,
  pitch=`2.8782`, yaw=`0.0`의 직접 static TF를 publish한다.

`radar → utlidar_lidar` edge는 만들지 않는다. joint state publisher, sensor driver,
RealSense·camera node, command·motion node, service, `/cmd_vel`, `/lowcmd`,
Sport API, `map`·`odom`도 포함하지 않는다.

실행은 설치 후 다음 명령을 사용한다.

```bash
ros2 launch bringup go2_static_tf.launch.py
```

2026-08-23 AGX에서 이 launch의 runtime 검증과 teardown을 수행했다. `/tf_static`
두 publisher와 세 핵심 edge를 확인했으며, `/tf` message와 `/joint_states`
publisher는 관찰되지 않았다. 이 결과는 독립 물리 calibration이나 동적 joint
상태 검증을 의미하지 않는다.

## `go2_odometry_adapter.launch.py`

이 launch는 `/utlidar/robot_odom` source를 읽는
`go2_state_estimation/odometry_adapter_node` 하나만 시작한다.

- `/odom`: `nav_msgs/msg/Odometry`, child frame `base`
- `/tf`: dynamic `odom → base`, adapter가 유일한 project owner
- source gate: `header.frame_id=odom`, `child_frame_id=base_link`, 양의 비감소
  timestamp, 유한한 pose·orientation·twist
- source covariance는 그대로 복사하며 all-zero covariance를 보정하지 않는다.
- `/cmd_vel`, `/lowcmd`, Sport API, service, motor, stand, walk는 실행하지 않는다.

실행은 설치 후 다음 명령을 사용한다.

```bash
ros2 launch bringup go2_odometry_adapter.launch.py
```

2026-08-23 AGX에서 source `10634`개 중 `10633`개를 `/odom`과 dynamic TF로
전달하고 `0`개를 거부했다. 종료 후 adapter publisher와 TF는 사라졌으며,
상세 결과는 `records/experiments/go2_local_navigation_odom_adapter_20260823.md`에
보존한다. 이 결과는 project-accepted `base_link → base` mapping의 runtime
검증이지, onboard frame의 공식 의미 동일성이나 물리 calibration 결과는 아니다.

## `go2_offline_rviz.launch.py`

이 launch는 Go2가 꺼져 있거나 Ethernet 연결이 없는 상태에서 URDF와 정적 TF를
RViz2로 확인하기 위한 시각화 전용 경계다.

- 기존 `go2_static_tf.launch.py`를 포함해 canonical URDF와
  `base → utlidar_lidar` static TF를 재사용한다.
- `joint_state_publisher`가 canonical URDF에서 합성 joint state를 publish해
  관절 link의 기본 자세를 표시한다. 이 값은 실제 Go2 관절 상태가 아니다.
- RViz2는 `RobotModel`, `TF`, `Grid`를 Fixed Frame `base`로 시작한다.
- CycloneDDS loopback 설정을 사용해 Go2 Ethernet interface가 없어도 시작된다.
- `/utlidar/*`, `/utlidar/robot_odom`, `/odom`, `/cmd_vel`, Unitree service와
  motor command를 사용하지 않는다.

실행은 설치 후 다음 명령을 사용한다.

```bash
ros2 launch bringup go2_offline_rviz.launch.py
```

이 launch에서 보이는 TF는 URDF와 프로젝트 static TF 기준이며, 실제 센서 위치·
현재 관절 자세·실시간 odometry 검증 결과가 아니다. RViz2 GUI가 표시되는
디스플레이 세션에서 실행해야 한다.

## `go2_stationary_perception.launch.py`

이 launch는 기존 `go2_static_tf.launch.py`를 포함하고
`go2_perception/obstacle_candidate_node` 하나만 시작한다.

- `/utlidar/cloud`를 `RELIABLE`, `KEEP_LAST(1)`, `VOLATILE`로 구독한다.
- node는 message timestamp에서 existing static TF를 lookup한 뒤 `base` frame의
  `/perception/obstacle_candidates` PointCloud2를 publish한다.
- 출력은 장애물 candidate일 뿐, 최종 분류나 free-space 증명이 아니다.
- odometry adapter, `/odom`, `/cmd_vel`, `/lowcmd`, Sport API, service, motor,
  stand, walk, RealSense는 시작하거나 사용하지 않는다.

실행은 설치 후 다음 명령을 사용한다.

```bash
ros2 launch bringup go2_stationary_perception.launch.py
```

2026-08-26 stationary bag E2E에서 raw cloud `300`개에 대해 candidate output
`300`개를 확인했다. timestamp 집합은 정확히 일치했고 output은 frame `base`,
fields `x/y/z`, finite point와 설정 경계 안의 값만 포함했다. 최종 clean-teardown
replay에서 모든 node가 정상 종료했고 잔류 project topic과 process는 없었다.
static TF 값은 이 launch가 소유하거나 복제하지 않으며, 기존 static TF launch가
유일한 owner다. 상세 결과는
`records/experiments/go2_local_navigation_stationary_bag_perception_20260826.md`에
보존한다.
