# go2_perception

실물 LiDAR를 읽어 base-frame obstacle candidate를 보고할 때 사용한다.
`ros2 run go2_perception obstacle_candidates`는 기존 TF를 message stamp로
조회하고 유한 point의 planar range 0.25~5.0 m, z -0.25~1.0 m를 유지한다.
출력 `/perception/obstacle_candidates`는 점유나 free-space의 증명이 아니다.

- 입력 `/utlidar/cloud`, frame `utlidar_lidar`, reliable/volatile KEEP_LAST(1).
- 출력 frame `base`, 원본 cloud stamp를 유지한다.
- `config/perception_contract.yaml`은 기존 관찰·미확인 경계를 보존한다.
- W04에서 mapping gate·scan profiles·mapping YAML/launch·전용 테스트는
  `src/go2_inputs`로 이동했다. 기존 scan 실행은
  `ros2 launch go2_inputs go2_mapping_scan.launch.py`로 바꾼다.

공유 cloud QoS는 `go2_inputs.cloud_qos`에서 가져온다. motion publisher,
service call과 물리 제어는 만들지 않는다. W04 실물 실행은 미검증이다.
