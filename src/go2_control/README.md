# `go2_control`

이 패키지는 Nav2의 평면 속도 후보를 프로젝트 제한값으로 제한하고 Unitree
`unitree_api/msg/Request` 형식으로 변환한다. 현재 기본 상태는 비동작 후보이며,
실제 Go2 제어 topic publisher는 두 개의 명시적 승인 parameter가 모두 참일 때만
생성된다.

## 현재 계약과 안전 경계

- 입력: `/go2_control/cmd_vel_candidate` (`geometry_msgs/msg/Twist`)
- preview: `/go2_control/sport_request_preview` (`unitree_api/msg/Request`)
- 실제 제어 후보: `/api/sport/request` (`unitree_api/msg/Request`)
- Move API ID: `1008`, parameter: `x`, `y`, `z`
- StopMove API ID: `1003`
- 실제 publisher gate: `output_enabled=true`와
  `physical_validation_approved=true`가 모두 필요
- 기본값: 두 parameter 모두 `false`
- `/lowcmd`, stand, walk, posture 전환, motion service call은 사용하지 않음

속도·가속도·timeout 기본값은 고정 전 비동작 검증을 위한 engineering candidate다.
공식 Move hard envelope나 실제 Go2 물리 적합값으로 승격하지 않는다. 축 방향, 제한값,
timeout 후 실제 정지 시간은 장비 고정 후 검증 대상으로 남긴다.

## 폴더 및 파일 구조

```text
go2_control/
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── go2_control
├── config/
│   └── motion_contract.yaml
├── go2_control/
│   ├── __init__.py
│   ├── motion_adapter_node.py
│   ├── motion_contract.py
│   └── sport_request.py
└── test/
    ├── test_motion_adapter_shutdown.py
    ├── test_motion_contract.py
    └── test_sport_request.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `README.md` | 입력·출력·승인 gate·검증 상태와 남은 물리 검증을 설명한다. |
| `package.xml` | ROS 2 package metadata와 `geometry_msgs`, `rclpy`, `unitree_api` 의존성을 선언한다. |
| `setup.py` | config 설치와 `motion_adapter` console entry point를 등록한다. |
| `setup.cfg` | 개발·설치 시 executable 경로를 지정한다. |
| `resource/go2_control` | ament index가 패키지를 찾기 위한 marker 파일이다. |
| `config/motion_contract.yaml` | 공식값·온보드 관찰·engineering candidate를 분리한 기계 검색용 계약이다. |
| `go2_control/__init__.py` | Python package 경계를 정의한다. |
| `go2_control/motion_adapter_node.py` | Twist 수신, 제한, watchdog, preview와 조건부 실제 publisher를 연결한다. |
| `go2_control/motion_contract.py` | ROS와 무관한 finite·속도·가속도·timeout·승인 gate 판단을 수행한다. |
| `go2_control/sport_request.py` | 제한된 command를 공식 Move·StopMove Request 필드로 변환한다. |
| `test/test_motion_adapter_shutdown.py` | ROS context 종료 뒤 중복 shutdown·logger 접근이 없는지 검사한다. |
| `test/test_motion_contract.py` | gate·finite·속도·가속도·timeout 경계를 검증한다. |
| `test/test_sport_request.py` | 공식 API ID와 Move parameter 형식을 검증한다. |

## 현재 단계에서의 사용법

현재 AGX에서는 `output_enabled=false`, `physical_validation_approved=false`를 유지한
비동작 dry-run과 통합 preflight까지 검증했다. 재실행에서도 두 gate를 닫아 preview만
관찰하며, 두 값을 참으로 바꾸는 것은 장비 고정과 최신 사용자 승인 이후의 물리 검증
범위다.
