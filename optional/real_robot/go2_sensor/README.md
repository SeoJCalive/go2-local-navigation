# go2_sensor

이 패키지는 AGX에서 관찰된 Go2 센서 topic의 type, frame, timestamp, field,
QoS 계약과 읽기 전용 LiDAR acceptance node를 소유한다. 2026-08-23 AGX에서
`/utlidar/cloud`의 기본 입력 계약을 확인했지만, 센서 외부 파라미터와 장애물
처리는 이 패키지의 범위가 아니다.

## 폴더 및 파일 구조

```text
go2_sensor/
├── LICENSE
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── go2_sensor
├── go2_sensor/
│   ├── __init__.py
│   ├── lidar_contract.py
│   └── lidar_acceptance_node.py
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    ├── test_pep257.py
    └── test_lidar_contract.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `LICENSE` | 라이선스 원문이며 패키지 동작 설명 파일은 아니다. |
| `README.md` | LiDAR 입력 계약, acceptance 실행 범위, 미확정 외부 파라미터와 안전 경계를 설명한다. |
| `package.xml` | ROS 2 패키지 이름·버전·의존성·빌드 형식을 선언한다. |
| `setup.py` | ament_python 설치 metadata와 `lidar_acceptance` console entry point를 정의한다. |
| `setup.cfg` | 개발·설치 시 script 경로를 패키지 이름에 맞춰 지정한다. |
| `resource/go2_sensor` | ament index가 패키지를 찾기 위한 빈 marker 파일이다. |
| `go2_sensor/__init__.py` | Python 패키지 경계와 모듈 설명을 제공한다. |
| `go2_sensor/lidar_contract.py` | ROS message에서 분리한 frame·field·timestamp 검증 규칙을 제공한다. |
| `go2_sensor/lidar_acceptance_node.py` | `/utlidar/cloud`를 읽기 전용으로 구독하고 계약 판정을 log로 보고한다. |
| `test/test_copyright.py` | 자동 생성된 copyright 검사다. |
| `test/test_flake8.py` | 자동 생성된 Python 형식 검사다. |
| `test/test_pep257.py` | 자동 생성된 docstring 규칙 검사다. |
| `test/test_lidar_contract.py` | frame·required field·timestamp 순서의 순수 계약을 검증한다. |

현재 acceptance node는 `/utlidar/cloud`만 구독한다. `x/y/z`가 포함된
`utlidar_lidar` frame과 timestamp 순서를 확인하지만, point cloud를 재발행하거나
TF·command·service·motor 제어를 수행하지 않는다.

## 현재 AGX 관찰 계약

- message type: `sensor_msgs/msg/PointCloud2`
- frame: `utlidar_lidar`
- fields: `x`, `y`, `z`, `intensity`, `ring`, `time`
- observed layout: height `1`, point step `32`, width examples `658`와 `679`
- width는 sample마다 달라질 수 있어 고정값으로 사용하지 않는다.
- publisher QoS: reliable, keep-last depth `1`, volatile
- latest short-window rate: 약 `15.404–15.423 Hz`
- read-only acceptance: valid `395`, invalid `0`
