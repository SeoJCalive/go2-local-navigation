# `go2_perception`

이 패키지는 Go2 로컬 내비게이션에서 센서 데이터를 읽고 장애물·가용 공간
정보로 변환하는 경계를 둔다. 초기 구성에서는 실행 노드와 명령 출력이
없다. `/utlidar/cloud`의 실제 PointCloud2 필드·QoS·좌표계 외부 파라미터를
확인한 뒤 구현한다.

## 입력과 출력 경계

- 입력 후보: `/utlidar/cloud` (`sensor_msgs/msg/PointCloud2`)
- 현재 상태: 관찰된 토픽과 주기는 있으나 필드·QoS·외부 파라미터는
  `unverified`이다.
- 출력 후보: 시각화 또는 후속 Nav2 입력용 읽기 전용 데이터
- 금지된 범위: `/cmd_vel`·스포츠 API·low-level motor command publish

상세 계약은 `bringup/config/sensor_contract.yaml`에 둔다.
측정하지 않은 센서 frame을 추측으로 `base_link`에 고정하지 않는다.

## 폴더 및 파일 구조

```text
go2_perception/
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── go2_perception
└── go2_perception/
    └── __init__.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `README.md` | perception의 입력·출력 경계와 아직 구현하지 않은 이유를 설명한다. |
| `package.xml` | ROS 2 metadata와 perception에 필요한 message·Python 의존성을 선언한다. |
| `setup.py` | ament_python 설치 metadata와 패키지 resource 등록을 정의한다. 실행 node는 등록하지 않는다. |
| `setup.cfg` | 개발·설치 시 script 경로를 지정한다. |
| `resource/go2_perception` | ament index가 패키지를 찾기 위한 빈 marker 파일이다. |
| `go2_perception/__init__.py` | Python 패키지 경계와 초기 scaffold 상태를 설명한다. |

현재 실제 perception node는 없다. 먼저 센서 계약과 frame 관계를 검증한 뒤
장애물·가용 공간 보고 구현을 추가한다.
