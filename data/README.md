# data

이 디렉터리는 센서 수용 검증용 rosbag과 실행 로그를 소스 패키지에서 분리해
보관한다. `.gitignore` 대상이므로 bag 원본은 Git에 포함하지 않고, 반복 참조할
경로·메시지 수·checksum은 저장소의 실행 record에 기록한다.

2026-08-26 정지 상태 검증 산출물은 다음과 같다.

- `bags/go2_stationary_raw_20260826_1829`: `/utlidar/cloud`와
  `/utlidar/robot_odom` source 원본
- `bags/go2_stationary_derived_20260826_1847`: `/odom`과
  `/perception/obstacle_candidates` replay 결과
- `runs/go2_stationary_perception_replay_20260826_1847`: runtime log
- `runs/go2_clean_teardown_replay_20260826_1858`: 종료 회귀 검증 log

정확한 수치와 checksum은
`records/experiments/go2_local_navigation_stationary_bag_perception_20260826.yaml`을
따른다.

2026-08-27 비동작 control·Nav2 산출물은 다음과 같다.

- `bags/go2_motion_dry_run_20260827`: candidate Twist 4개와 Sport Request preview 6개
- `bags/go2_nav2_controller_preview_20260827`: candidate, preview, local costmap,
  obstacle candidate, odometry와 path 총 10205개 message

정확한 topic별 수치와 checksum은 각각
`records/experiments/go2_local_navigation_motion_adapter_dry_run_20260827.yaml`,
`records/experiments/go2_local_navigation_nav2_non_actuating_preview_20260827.yaml`을
따른다.

같은 날 9단계 통합 비동작 preflight 산출물은 다음 경로에 있다.

- `runs/preflight/20260827_020502_stage9/result.json`: host·observer·teardown을 합친
  최종 46개 check 결과
- `runs/preflight/20260827_020502_stage9/observer.json`: ROS graph·TF·topic·gate 관찰
- `runs/preflight/20260827_020502_stage9/launch.log`: 전체 launch와 clean exit 로그
- `runs/preflight/20260827_020502_stage9/tegrastats.log`: 30초 AGX 자원 표본

최종 판정, 수치와 checksum은
`records/experiments/go2_local_navigation_integrated_preflight_20260827.yaml`을 따른다.
