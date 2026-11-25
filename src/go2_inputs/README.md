# go2_inputs

SLAM·AMCL이 소비할 읽기 전용 cloud/scan/odometry 입력을 조합할 때 사용한다.
기존 perception의 mapping gate와 state-estimation의 adapter를 그대로 옮겼다.
`description`의 canonical static TF를 포함하고 공식 `pointcloud_to_laserscan`을
사용한다. `/scan`, `/odom`, `odom → base`의 topic·frame·QoS·기본 parameter와
simulation clock reset 처리는 W06과 같다. 실물 probe와 obstacle candidate는
`optional/real_robot` 패키지가 소유한다.

- `go2_mapping_scan.launch.py`: static TF, mapping cloud gate, scan converter.
- `go2_odometry_adapter.launch.py`: adapter 하나, 기본 `onboard_observe`.
- `mapping_cloud_gate`, `odometry_adapter`: 설치 실행 파일.
- `config/`: 이동한 scan·odometry 계약의 단일 원본.
- 의존 방향: `description`만 workspace runtime dependency다.

설치된 share와 Python namespace는 `go2_inputs`다. 기존 호출은
`go2_perception.mapping_*`, `go2_state_estimation`의 adapter/contract/epoch 모듈
대신 `go2_inputs`를 사용한다. 실물 입력도 같은 adapter를 재사용하며 motion
publisher는 만들지 않는다.
