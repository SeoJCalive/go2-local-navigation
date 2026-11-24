# bringup

실물·비동작 입력과 offline RViz를 조합할 때 사용한다. static TF는
`description/go2_static_tf.launch.py`, odometry는
`go2_inputs/go2_odometry_adapter.launch.py`를 포함한다.

- `go2_sensor_acceptance.launch.py`: 센서 acceptance와 source probe.
- `go2_stationary_perception.launch.py`: static TF와 obstacle candidate.
- `go2_offline_rviz.launch.py`: canonical URDF·합성 joint state·RViz.
- `go2_costmap_only.launch.py`: 비동작 local costmap.
- `go2_controller_preview.launch.py`: 닫힌 Sport gate까지 controller 후보 연결.
- `config/nav2_non_actuating.yaml`: 위 두 Nav2 launch의 설정 원본.
- `config/sensor_contract.yaml`: 과거 실제 관찰과 미확인 조건.

W04에서 preflight·fault·mode·TF-owner observer와 helper는
`tools/go2_validation`으로 이동했다. 설치 executable은 없다.
실제 command gate·토픽·기본 설정은 변경하지 않았다. W04는 실물 경로를
build/test만 검증하며 최신 실물 실행 승인을 대신하지 않는다.
