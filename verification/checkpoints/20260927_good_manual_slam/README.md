# 2026-09-27 수동 SLAM 체크포인트

이 폴더는 AGX `DISPLAY=:0`에서 Xbox 패드로 직접 주행하며 지도 품질이 양호하게 보였던 시점의 SLAM 결과를 보존한다. 지도는 사용자의 관찰에 따라 좋은 상태로 선택한 스냅샷이며, 실제 환경 ground truth 대비 정확도나 이후 loop closure 안정성이 입증된 것은 아니다.

- 환경: `go2_complex_indoor.xml`, 공식 `unitree_mujoco` Go2, ROS 2 Humble Domain 1 loopback. 같은 geometry의 월드 스냅샷을 이 폴더에도 보존했다. 실행 당시 원본의 SHA-256은 `f164137d1b61d2c113df2e6f9fe9f0bb096f5c5085ec34547a2ed6e38ed39f2d`이다.
- 입력: MuJoCo 전용 `go2_mujoco_lidar`의 planar-selected cloud → `/scan`; `/utlidar/robot_odom` → `/odom`.
- LiDAR 실행값: 수평 `720`, 수직 `8`, 수직각 `-0.2617993877991494`부터 `0.2617993877991494` rad.
- SLAM: 기존 `go2_nav2/go2_slam_mapping.launch.py`, `start_inputs:=false`, `use_sim_time:=true`; 기본 `slam_mapping.yaml`과 loop-closure 설정은 변경하지 않음.
- 조작: MuJoCo `joystick_type: switch`, `/dev/input/js0`를 Xbox 패드로 사용; RL controller는 `Velocity_Down` 상태에서 수동 주행.
- 저장물: 2026-09-27 23:57:59 KST에 저장한 occupancy map `map.pgm`·`map.yaml`(320×245셀, 0.05 m/셀)과 23:58:03 KST에 직렬화한 SLAM Toolbox pose graph `posegraph.posegraph`·`posegraph.data`.

Git 커밋은 코드·실행 조건의 기준점이고, 저장 지도는 이 특정 주행 시점의 결과다. 저장 지도를 새 SLAM의 입력으로 자동 재사용하지 않는다.
두 저장 호출 간 약 4초가 경과했으므로 occupancy와 pose graph가 정확히 같은 frame의 원자적 스냅샷은 아니다.
