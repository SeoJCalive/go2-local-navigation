# go2_state_estimation

이 패키지는 AGX의 `/utlidar/robot_odom` source 계약을 관찰하고, 프로젝트에서
수용한 frame mapping이 적용된 경우 `/odom`과 `odom → base` dynamic TF를 제공한다.
source probe는 계속 읽기 전용이며, adapter는 유효한 source sample만 출력한다.

현재 source child frame은 `base_link`로 관찰됐고 프로젝트 canonical target frame은
`base`다. 이 패키지는 `base_link → base`를 프로젝트 통합 mapping으로 수용한다.
이는 공식 onboard frame 정의나 독립 물리 calibration의 확정이 아니며, adapter는
source child가 정확히 `base_link`일 때만 동작한다.

## 파일 구조와 책임

```text
go2_state_estimation/
├── config/odometry_contract.yaml       # source와 project adapter의 odometry 계약
├── go2_state_estimation/
│   ├── odometry_contract.py            # ROS 의존성이 없는 검증 규칙
│   ├── odometry_adapter_node.py        # source를 /odom과 odom → base로 전달
│   └── odometry_probe_node.py          # Odometry subscription과 logger 보고
├── resource/go2_state_estimation       # ament package resource marker
├── test/test_odometry_contract.py      # 순수 계약 unit test
├── package.xml                          # ament_python·ROS runtime 의존성
└── setup.py                             # 설치 metadata와 probe·adapter console entry point
```

## 실행 경계

- 입력: `/utlidar/robot_odom` (`nav_msgs/msg/Odometry`)만 구독한다.
- QoS: `RELIABLE`, `KEEP_LAST(1)`, `VOLATILE`.
- `odometry_probe`: logger만 사용하며 source child frame과 covariance 상태를 기록한다.
- `odometry_adapter`: `/odom`에 child `base`인 `nav_msgs/msg/Odometry`를 publish하고,
  같은 pose·timestamp로 dynamic `odom → base` TF를 publish한다.
- adapter는 `/cmd_vel` publish, service call, motion을 수행하지 않으며, 잘못된
  source frame·timestamp·수치 sample은 출력하지 않는다.

이 README와 `config/odometry_contract.yaml`은 source 관찰과 project adapter의
실행 계약을 설명한다. `odom → base` owner는 `odometry_adapter_node` 하나로
정한다. 공식 reference와 project mapping의 구분은 결정 record에 보존한다.

2026-08-23 AGX에서 read-only probe는 `3851`개 sample을 받아 invalid `0`으로
확인했고, adapter runtime은 `10634`개를 받아 `10633`개를 publish하고 `0`개를
reject했다. `/odom` publisher와 dynamic `odom → base` TF를 확인한 뒤 두 launch를
teardown했으며, command topic baseline은 변하지 않았다. all-zero covariance는
source 값을 그대로 전달했고 수치 보정하지 않았다.

상세 결과는
`records/experiments/go2_local_navigation_odom_adapter_20260823.md`와 같은 stem의
YAML projection에서 확인한다.
