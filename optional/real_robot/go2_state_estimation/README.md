# go2_state_estimation

실물 `/utlidar/robot_odom`의 frame·stamp·속도·공분산 계약을 읽기 전용으로
관찰할 때 `ros2 run go2_state_estimation odometry_probe`를 사용한다.
probe는 `go2_inputs.odometry_contract`의 기존 순수 판정 규칙을 재사용한다.

W04에서 adapter·continuity profiles·simulation epoch helper와 YAML·테스트는
`src/go2_inputs`로 이동했다. adapter 직접 실행은
`ros2 run go2_inputs odometry_adapter`, launch는
`ros2 launch go2_inputs go2_odometry_adapter.launch.py`다.
source frame `odom/base_link`에서 canonical `odom/base`로의 통합 mapping과
QoS·값·기본 profile은 유지한다. 이 mapping은 물리 calibration의 확정이 아니다.
