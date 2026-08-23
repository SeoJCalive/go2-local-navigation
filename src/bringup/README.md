# `bringup`

이 패키지는 센서 입력 계약과 ROS 2 launch 구성을 관리한다. 현재 실행 가능한
launch는 static TF, read-only acceptance, odometry adapter 세 종류다. 설치된
`description` 패키지의 canonical URDF를 사용해 모델 TF와 승인된 직접
`base → utlidar_lidar` static TF를 기동하고, 별도 adapter launch는
`odom → base` 동적 TF와 `/odom`만 추가한다. 2026-08-23 AGX에서 세 launch의
frame·topic 조회, 안전 경계, teardown을 검증했다.

## 현재 범위

- `/utlidar/cloud`, `/utlidar/imu`, `/lf/lowstate`의 관찰 정보 보존
- 메시지 타입·프레임 이름·주기·확인 상태의 명시
- `description` URDF용 `robot_state_publisher` 1개와 `base → utlidar_lidar`
  `static_transform_publisher` 1개만 시작
- `go2_odometry_adapter.launch.py`는 `/utlidar/robot_odom`을 읽고 `/odom`과
  `odom → base` dynamic TF를 추가하며, source frame mapping은
  `go2_state_estimation` 계약에 따른다.
- `radar → utlidar_lidar` TF, sensor driver, camera, command topic, service call,
  motion·navigation node를 시작하지 않음
- `robot_state_publisher`가 광고하는 `/tf` publisher와 `/joint_states`
  subscription은 존재할 수 있지만, 이 launch는 joint state를 publish하지 않으며
  검증 시 `/tf` message와 `/joint_states` publisher가 0개인지 확인한다.

각 launch의 실제 AGX runtime 결과와 중지 결과는 실행 record에 보존한다.
이전 관찰값만으로 sensor driver, TF 추가 edge, motion 경로를 자동 기동하지 않는다.

2026-08-23 AGX에서 `go2_sensor_acceptance.launch.py`를 실행해 LiDAR valid `395`,
invalid `0`, odometry received `3851`, invalid `0`을 확인했다. acceptance launch는
`/tf`, `/tf_static`, `/odom`, `/cmd_vel`을 publish하지 않았으며, 상세 결과는
`records/experiments/go2_local_navigation_sensor_odometry_probe_20260823.md`에
기록했다.

같은 날 `go2_odometry_adapter.launch.py`는 source `10634`개 중 `10633`개를
`/odom`과 `odom → base`로 전달했고, `/tf`의 dynamic owner를 확인했다. 종료 후
adapter가 publish하던 topic·TF는 사라졌고 command topic baseline은 변하지 않았다.
상세 결과는 `records/experiments/go2_local_navigation_odom_adapter_20260823.md`에
기록했다.

## 폴더 및 파일 구조

```text
bringup/
├── LICENSE
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── bringup
├── config/
│   └── sensor_contract.yaml
├── launch/
│   ├── go2_sensor_acceptance.launch.py
│   ├── go2_odometry_adapter.launch.py
│   ├── go2_static_tf.launch.py
│   └── README.md
├── bringup/
│   └── __init__.py
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `LICENSE` | 라이선스 원문이며 bringup 동작 설명 파일은 아니다. |
| `README.md` | static TF, read-only acceptance, odometry adapter launch의 범위와 runtime 검증 결과를 설명한다. |
| `package.xml` | ROS 2 metadata와 launch 의존성을 선언한다. |
| `setup.py` | package metadata와 `config/*.yaml`, 향후 `launch/*.py` 설치 경로를 정의한다. 실행 node는 등록하지 않는다. |
| `setup.cfg` | 개발·설치 시 script 경로를 지정한다. |
| `resource/bringup` | ament index가 패키지를 찾기 위한 빈 marker 파일이다. |
| `config/sensor_contract.yaml` | AGX에서 관찰한 topic·message type·주기·frame 상태와 source·adapter 안전 경계를 구조화한다. |
| `launch/go2_sensor_acceptance.launch.py` | TF나 command를 publish하지 않고 LiDAR acceptance와 odometry source probe만 시작한다. |
| `launch/go2_odometry_adapter.launch.py` | `/utlidar/robot_odom`을 `/odom`과 `odom → base` dynamic TF로 전달하는 adapter 하나만 시작한다. |
| `launch/go2_static_tf.launch.py` | 설치된 `description` URDF로 `robot_state_publisher` 하나와 직접 `base → utlidar_lidar` static TF publisher 하나만 시작한다. |
| `launch/README.md` | static TF, read-only acceptance, odometry adapter launch의 실행 범위와 runtime 검증 결과를 설명한다. |
| `bringup/__init__.py` | Python 패키지 경계를 정의하며 executable node를 등록하지 않는다. |
| `test/test_copyright.py` | 자동 생성된 copyright 검사다. |
| `test/test_flake8.py` | 자동 생성된 Python 형식 검사다. |
| `test/test_pep257.py` | 자동 생성된 docstring 규칙 검사다. |
