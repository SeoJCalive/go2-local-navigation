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
