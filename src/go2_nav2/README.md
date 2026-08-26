# `go2_nav2`

이 패키지는 현재 프로젝트의 Nav2 local costmap·controller와 전체 비동작 stack
runner를 소유한다. 기존 센서·TF·odometry·motion 변환 코드는 복제하지 않고 각
패키지의 실행 경계를 조합한다.

## 폴더 및 파일 구조

```text
go2_nav2/
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── go2_nav2
├── config/
│   ├── navigation_contract.yaml
│   └── nav2_non_actuating.yaml
├── launch/
│   ├── go2_costmap_only.launch.py
│   ├── go2_controller_preview.launch.py
│   └── go2_integrated_preflight.launch.py
├── go2_nav2/
│   ├── __init__.py
│   ├── preflight_runner_configuration.py
│   └── preflight_runner_node.py
└── test/
    ├── test_navigation_configuration.py
    └── test_preflight_package_ownership.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `README.md` | 비동작 Nav2·통합 preflight 구성, 입력·출력, 검증 범위와 물리 미확인 항목을 설명한다. |
| `package.xml` | Nav2 controller·costmap·DWB, `bringup`과 기존 프로젝트 패키지의 실행 의존성을 선언한다. |
| `setup.py` | config와 launch를 ROS 2 share 경로에 설치하고 `integrated_preflight` entry point를 등록한다. |
| `setup.cfg` | Python 실행 파일 설치 경로를 지정한다. |
| `resource/go2_nav2` | ament index가 패키지를 찾는 marker다. |
| `config/navigation_contract.yaml` | 공식 근거·AGX 설치 관찰·engineering candidate·미확인을 분리한다. |
| `config/nav2_non_actuating.yaml` | local costmap과 controller server가 실제로 읽는 parameter다. |
| `launch/go2_costmap_only.launch.py` | static TF·odometry·obstacle candidate와 비동작 local costmap owner만 시작한다. |
| `launch/go2_controller_preview.launch.py` | controller 출력을 내부 candidate topic으로 보내고 gate가 닫힌 adapter preview까지 연결한다. |
| `launch/go2_integrated_preflight.launch.py` | 필수 비동작 stack과 시간 제한 observer를 함께 시작하고 observer 완료 뒤 전체 launch를 종료한다. |
| `go2_nav2/__init__.py` | 비동작 Nav2 조합과 통합 runner의 Python package 경계를 설명한다. |
| `go2_nav2/preflight_runner_configuration.py` | 실행 시간·label·산출물 경로 ROS parameter를 안전한 실행 구성으로 파싱한다. |
| `go2_nav2/preflight_runner_node.py` | 통합 launch, `tegrastats`, kernel 관찰과 clean teardown을 순서대로 실행해 최종 JSON을 만든다. |
| `test/test_navigation_configuration.py` | frame·obstacle source·controller plugin·candidate 제한값을 검사한다. |
| `test/test_preflight_package_ownership.py` | 통합 runner가 이 패키지에 있고 `bringup → go2_nav2` 순환 의존성이 없는지 검사한다. |

## 데이터 흐름

costmap-only 경로는 다음과 같다.

```text
/utlidar/robot_odom → /odom + odom → base
/utlidar/cloud → /perception/obstacle_candidates
TF + obstacle candidates → /local_costmap/costmap
controller output → /go2_nav2/costmap_only_cmd_vel_unused
```

costmap-only launch는 Humble standalone costmap의 종료 결함을 피하기 위해 lifecycle
정리가 가능한 `controller_server`가 local costmap을 소유한다. motion adapter와 action
goal은 시작하지 않으며 controller output은 소비자가 없는 격리 topic으로 보낸다.

controller preview 경로는 다음과 같다.

```text
odom frame FollowPath
  → controller_server
  → /go2_control/cmd_vel_candidate
  → go2_motion_adapter
  → /go2_control/sport_request_preview
  ╳ /api/sport/request
```

`go2_controller_preview.launch.py`는 `output_enabled=false`와
`physical_validation_approved=false`를 직접 지정한다. Nav2의 일반 `cmd_vel`은
프로젝트 내부 candidate topic으로 remap하며 실제 Sport control publisher를 만들지
않는다.

통합 preflight 경로는 다음과 같다.

```text
integrated_preflight runner
  → go2_integrated_preflight.launch.py
  → static TF + odometry + obstacle candidate + controller + closed motion adapter
  → bringup/integrated_preflight_observer
  → result.json + launch.log + tegrastats.log
```

runner는 observer가 기록한 graph·TF·topic·gate 결과에 host 자원, kernel event와
teardown 결과를 합친다. 실행 중 project owner의 `/api/sport/request`·`/lowcmd`
publisher가 하나라도 관찰되면 실패한다.

```bash
ros2 run go2_nav2 integrated_preflight \
  --ros-args -p duration_sec:=30 -p run_label:=stage9
```

## AGX 검증 상태

2026-08-27 costmap-only와 controller preview를 정지 상태에서 실행했다. 두 경로 모두
`controller_server`와 local costmap이 active가 됐고 `/local_costmap/costmap`은 약
`1.667 Hz`였다. controller preview의 0.30 m 시험 경로는 로봇이 움직이지 않아 5초
progress checker에서 예상대로 aborted됐다.

motion adapter의 `/api/sport/request` publisher가 없고 두 gate가 false임을 확인했다.
최종 종료에서는 odometry adapter를 포함한 모든 process가 clean exit했다. standalone
`nav2_costmap_2d` 1.1.20은 lifecycle 종료 뒤에도 class-loader signal 종료가 발생해
현재 launch 경로에서 제외했다.

같은 날 30초 통합 preflight run `20260827_020502_stage9`에서 46개 check가 모두
PASS였다. 필수 node 7개와 TF 5개, 실제 센서·odom·obstacle candidate·local costmap,
닫힌 두 gate, 최대 온도 `42.718°C`와 잔류 process 0개를 확인했다. 이 결과는 30분
soak나 물리 navigation 검증이 아니다.

## 현재 candidate 경계

- Nav2 기준 frame: `odom`, robot frame: `base`
- obstacle source: `/perception/obstacle_candidates`, `PointCloud2`
- obstacle 처리: marking만 사용하며 clearing과 free-space 주장은 하지 않음
- controller plugin: 설치된 Humble `dwb_core::DWBLocalPlanner`
- 속도·가속도 상한: `go2_control` candidate 이하
- robot radius `0.30 m`: 비동작 costmap 연결용 candidate
- planner, SLAM, localization, `map → odom`, 실제 목적지 도달은 현재 범위가 아님

공식 parameter 구조는 Nav2 Humble의
[nav2_params.yaml](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)을
기준으로 하고, plugin 이름은 AGX에 설치된 `1.1.20` package 선언과 대조한다.
