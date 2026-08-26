# `go2_perception`

이 패키지는 Go2 로컬 내비게이션에서 정지 상태 LiDAR를 읽어 base-frame
obstacle candidate PointCloud2로 보고하는 읽기 전용 경계다. candidate는 최종
장애물 분류도 free-space 증명도 아니다.

## 입력과 출력 경계

- 입력: `/utlidar/cloud` (`sensor_msgs/msg/PointCloud2`), `RELIABLE`,
  `KEEP_LAST(1)`, `VOLATILE`, frame `utlidar_lidar`
- TF: node가 message timestamp로 기존 static TF tree를 lookup해 `base`로
  XYZ를 변환한다. perception은 extrinsic 수치를 복제하거나 추정하지 않는다.
- 필터: 변환 뒤 유한한 point만 planar range `[0.25, 5.0]` m 및 base-frame z
  `[-0.25, 1.0]` m의 포함 경계로 남긴다.
- 출력: `/perception/obstacle_candidates` (`sensor_msgs/msg/PointCloud2`),
  input header timestamp를 유지하고 frame은 `base`다.
- 금지된 범위: `/cmd_vel`, `/lowcmd`, Sport API, motor, stand, walk,
  service call, RealSense 접근

`config/perception_contract.yaml`은 project-observed 입력과 project canonical
target을 구분한다. 2026-08-26 stationary bag replay에서 runtime output과
teardown을 검증했지만 candidate 의미는 최종 장애물 분류로 승격하지 않는다.

## 폴더 및 파일 구조

```text
go2_perception/
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── go2_perception
├── config/
│   └── perception_contract.yaml
├── go2_perception/
│   ├── __init__.py
│   ├── obstacle_candidate_node.py
│   └── perception_contract.py
└── test/
    ├── test_obstacle_candidate_shutdown.py
    └── test_perception_contract.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `README.md` | stationary perception 입력·출력, candidate 의미와 안전 경계를 설명한다. |
| `package.xml` | ROS 2 metadata와 perception에 필요한 message·Python 의존성을 선언한다. |
| `setup.py` | ament_python metadata, config 설치와 `obstacle_candidates` executable을 등록한다. |
| `setup.cfg` | 개발·설치 시 script 경로를 지정한다. |
| `resource/go2_perception` | ament index가 패키지를 찾기 위한 빈 marker 파일이다. |
| `config/perception_contract.yaml` | project-observed 입력, project target, filter와 candidate 상태를 구조화한다. |
| `go2_perception/__init__.py` | Python package 경계와 candidate 의미를 설명한다. |
| `go2_perception/obstacle_candidate_node.py` | TF lookup, PointCloud2 변환·필터·candidate publish만 수행하는 ROS node다. |
| `go2_perception/perception_contract.py` | ROS 의존성 없는 point 변환과 candidate filter 계약이다. |
| `test/test_obstacle_candidate_shutdown.py` | ROS context 종료 뒤 중복 shutdown·logger 접근이 없는지 검사한다. |
| `test/test_perception_contract.py` | contract, quaternion geometry, range·z·finite filter 순수 unit test다. |

bringup의 `go2_stationary_perception.launch.py`는 기존 static TF launch와 이
node만 시작한다. stationary bag E2E에서 raw cloud `300`개와 output `300`개의
timestamp가 정확히 일치했고, output은 모두 frame `base`, fields `x/y/z`, finite
point와 설정 경계 안의 값으로 확인됐다. 상세 수치는
`records/experiments/go2_local_navigation_stationary_bag_perception_20260826.md`에
보존한다.
