# `bringup`

이 패키지는 센서 입력 계약과 ROS 2 launch 구성을 관리한다. 현재 실행 가능한
launch는 static TF, read-only acceptance, odometry adapter, stationary perception,
offline RViz 다섯 종류다. 설치된
`description` 패키지의 canonical URDF를 사용해 모델 TF와 선택된 profile의 직접
`base → utlidar_lidar` static TF를 기동하고, 별도 adapter launch는
`odom → base` 동적 TF와 `/odom`만 추가한다. 2026-08-23 AGX에서 세 launch의
frame·topic 조회, 안전 경계, teardown을 검증했다. 2026-08-26에는 stationary
perception launch의 bag replay와 clean teardown을 추가로 검증했다. 패키지 내부의
preflight 모듈과 observer는 이 입력·TF·안전 경계를 통합 관찰할 때 재사용하며, 전체
stack 조합과 host lifecycle은 `go2_validation` runner가 소유한다.
fault scenario schema, 실행 mode별 domain 계약과 global TF owner 판정도 이
패키지의 공통 계약으로 관리한다.

## 현재 범위

- `/utlidar/cloud`, `/utlidar/imu`, `/lf/lowstate`의 관찰 정보 보존
- 메시지 타입·프레임 이름·주기·확인 상태의 명시
- `description` URDF용 `robot_state_publisher` 1개와 `base → utlidar_lidar`
  `static_transform_publisher` 1개만 시작한다. 기본 `project_default`는 기존 값을
  유지하고 `dimos_replay`는 DimOS 외부 replay에서만 명시한다.
- `go2_odometry_adapter.launch.py`는 `/utlidar/robot_odom`을 읽고 `/odom`과
  `odom → base` dynamic TF를 추가하며, source frame mapping은
  `go2_state_estimation` 계약에 따른다. 기본 continuity profile은
  `onboard_observe`이며 replay caller만 `replay_enforce`를 명시한다.
- `go2_offline_rviz.launch.py`는 로봇 연결 없이 canonical URDF, static TF,
  합성 joint state와 RViz2를 시작한다. live odom·센서·command는 사용하지 않는다.
- `go2_stationary_perception.launch.py`는 기존 static TF launch와
  `/utlidar/cloud` read-only obstacle candidate node만 시작한다. odometry adapter,
  motion과 command interface는 포함하지 않는다.
- `integrated_preflight_observer`는 필수 graph·TF·topic과 닫힌 motion gate를 시간
  제한으로 관찰해 JSON을 만들며, host 자원·process lifecycle은 소유하지 않는다.
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

2026-08-26 `go2_stationary_perception.launch.py`의 격리 bag replay에서 raw cloud
`300`개를 frame `base`의 candidate cloud `300`개로 변환했다. `/cmd_vel`과 command
interface는 생성되지 않았으며 최종 teardown 뒤 project process와 topic은 남지
않았다. 상세 결과는
`records/experiments/go2_local_navigation_stationary_bag_perception_20260826.md`에
기록했다.

2026-08-27 통합 비동작 preflight에서 observer는 필수 node 7개, TF 5개와 7개 topic
계약을 30초 동안 확인했다. 두 gate는 false였고 프로젝트 소유
`/api/sport/request`·`/lowcmd` publisher는 최대 0개였다. runner가 합친 46개 check는
모두 PASS였으며 상세 결과는
`records/experiments/go2_local_navigation_integrated_preflight_20260827.md`에
기록했다.

같은 날 30초 smoke는 47 PASS였고 30분 soak는 46 PASS·1 WARN이었다. 필수 topic,
자원, kernel, 닫힌 gate와 teardown은 통과했지만 정지 yaw 누적 drift
`0.248012 rad`가 경고로 남았다. 상세 결과는
`records/experiments/go2_local_navigation_stationary_soak_20260827.md`에 기록했다.

2026-08-28 DimOS source extrinsic을 `dimos_replay` profile로 추가하고 동일 120초
bag에서 기존 `project_default`와 A/B했다. source-aligned profile은 jump 수치를
줄였지만 두 profile 모두 `map → odom` 연속성 기준을 초과했다. 따라서 default를
교체하지 않았고 replay profile도 실물 calibration으로 승격하지 않는다. 상세 결과는
`records/experiments/go2_local_navigation_dimos_tf_profile_ab_20260828.md`에 기록했다.

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
│   ├── fault_scenarios.yaml
│   ├── sensor_contract.yaml
│   └── static_tf_profiles.yaml
├── launch/
│   ├── go2_sensor_acceptance.launch.py
│   ├── go2_odometry_adapter.launch.py
│   ├── go2_static_tf.launch.py
│   ├── go2_offline_rviz.launch.py
│   ├── go2_stationary_perception.launch.py
│   └── README.md
├── rviz/
│   └── go2_offline_model.rviz
├── bringup/
│   ├── __init__.py
│   ├── fault_contract.py
│   ├── fault_result.py
│   ├── mode_observer.py
│   ├── preflight_accumulator.py
│   ├── preflight_assessments.py
│   ├── preflight_configuration.py
│   ├── preflight_host.py
│   ├── preflight_metrics.py
│   ├── preflight_observer_node.py
│   ├── preflight_report.py
│   ├── preflight_resources.py
│   ├── preflight_result.py
│   ├── preflight_ros_samples.py
│   ├── preflight_subscriptions.py
│   ├── preflight_types.py
│   ├── static_tf_profiles.py
│   └── tf_owner_audit.py
└── test/
    ├── test_copyright.py
    ├── test_fault_contract.py
    ├── test_flake8.py
    ├── test_mode_observer.py
    ├── test_pep257.py
    ├── test_preflight_command_ownership.py
    ├── test_preflight_metrics.py
    ├── test_preflight_resources.py
    ├── test_preflight_runtime_boundaries.py
    ├── test_static_tf_profiles.py
    └── test_tf_owner_audit.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `LICENSE` | 라이선스 원문이며 bringup 동작 설명 파일은 아니다. |
| `README.md` | static TF, read-only acceptance, adapter, perception과 통합 observer의 범위와 runtime 검증 결과를 설명한다. |
| `package.xml` | ROS 2 metadata, observer message type과 launch 의존성을 선언한다. |
| `setup.py` | sensor·fault·static TF profile config, launch·RViz 설치 경로와 `integrated_preflight_observer` entry point를 정의한다. |
| `setup.cfg` | 개발·설치 시 script 경로를 지정한다. |
| `resource/bringup` | ament index가 패키지를 찾기 위한 빈 marker 파일이다. |
| `config/fault_scenarios.yaml` | software fault 10종의 기대 차단·복구 조건을 정의한다. |
| `config/sensor_contract.yaml` | AGX에서 관찰한 topic·message type·주기·frame 상태와 source·adapter 안전 경계를 구조화한다. |
| `config/static_tf_profiles.yaml` | 기존 실물 기본값과 DimOS 외부 replay 전용 static sensor TF를 출처·적용 범위와 함께 분리한다. replay-only profile은 `execution_mode=external_replay`에서만 선택된다. |
| `launch/go2_sensor_acceptance.launch.py` | TF나 command를 publish하지 않고 LiDAR acceptance와 odometry source probe만 시작한다. |
| `launch/go2_odometry_adapter.launch.py` | `/utlidar/robot_odom`을 `/odom`과 `odom → base` dynamic TF로 전달하는 adapter 하나만 시작하고 continuity profile을 parameter로 넘긴다. |
| `launch/go2_static_tf.launch.py` | 설치된 `description` URDF와 실행 mode에 허용된 `sensor_tf_profile`로 `robot_state_publisher` 하나와 직접 `base → utlidar_lidar` static TF publisher 하나만 시작한다. |
| `launch/go2_offline_rviz.launch.py` | 로봇 연결 없이 static TF launch, 합성 joint state publisher, RViz2를 함께 시작한다. |
| `launch/go2_stationary_perception.launch.py` | 기존 static TF launch와 `go2_perception` obstacle candidate node만 시작한다. |
| `launch/README.md` | static TF, read-only acceptance, odometry adapter, stationary perception launch의 실행 범위와 runtime 검증 결과를 설명한다. |
| `rviz/go2_offline_model.rviz` | Fixed Frame `base`의 RobotModel·TF·Grid 기본 표시 설정이다. live topic display는 포함하지 않는다. |
| `bringup/__init__.py` | launch와 재사용 가능한 preflight observer의 Python 패키지 경계를 설명한다. |
| `bringup/fault_contract.py` | fault YAML을 폐쇄된 scenario 타입으로 파싱하고 잘못된 상태를 거부한다. |
| `bringup/fault_result.py` | scenario별 결과와 Stage 11 JSON report 구조를 정의한다. |
| `bringup/mode_observer.py` | 실행 mode별 domain·network·global TF owner 계약을 판정한다. |
| `bringup/preflight_accumulator.py` | 고빈도 callback에서 frame·timestamp·rate·정지 pose 통계를 고정 크기로 누적한다. |
| `bringup/preflight_assessments.py` | 환경·graph·TF·gate와 프로젝트 소유 command publisher를 독립 check로 판정한다. |
| `bringup/preflight_configuration.py` | 필수 topic·node·TF, timing과 정지 pose 후보 기준을 정의한다. |
| `bringup/preflight_host.py` | AGX thermal trip·kernel event·잔류 process를 읽고 runner 소유 process를 종료한다. |
| `bringup/preflight_metrics.py` | topic 연속성·schema·timing·정지 drift와 전체 상태를 판정한다. |
| `bringup/preflight_observer_node.py` | ROS graph·TF·topic·gate를 시간 제한으로 관찰해 `observer.json`을 만든다. |
| `bringup/preflight_report.py` | observer 관찰값의 불변 구조와 JSON provenance를 정의한다. |
| `bringup/preflight_resources.py` | `tegrastats`와 kernel event를 시작·종료 RAM/온도 추세, maximum memory·thermal과 OOM check로 변환한다. |
| `bringup/preflight_result.py` | observer JSON 경계를 검증하고 runner의 최종 result 저장을 지원한다. |
| `bringup/preflight_ros_samples.py` | ROS message를 고정 크기의 type·frame·timestamp·pose 표본으로 변환한다. |
| `bringup/preflight_subscriptions.py` | 필수 ROS topic type, QoS와 표본 변환기를 observer에 연결한다. |
| `bringup/preflight_types.py` | check status, topic contract, pose와 summary의 공유 불변 타입을 정의한다. |
| `bringup/static_tf_profiles.py` | profile registry의 frame·vector·정규화 quaternion을 파싱하고 unknown·invalid profile을 launch 전에 거부한다. |
| `bringup/tf_owner_audit.py` | `/tf` transform과 publisher GID를 결합해 global edge owner 수와 node를 판정한다. |
| `test/test_copyright.py` | 자동 생성된 copyright 검사다. |
| `test/test_fault_contract.py` | fault manifest와 결과 schema를 검사한다. |
| `test/test_flake8.py` | 자동 생성된 Python 형식 검사다. |
| `test/test_mode_observer.py` | offline/live mode의 domain·network 환경 판정을 검사한다. |
| `test/test_pep257.py` | 자동 생성된 docstring 규칙 검사다. |
| `test/test_preflight_command_ownership.py` | 기존 bare DDS endpoint를 프로젝트 command publisher로 세지 않는지 검사한다. |
| `test/test_preflight_metrics.py` | topic 연속성·timestamp 역행·정지 drift와 전체 상태 판정을 검사한다. |
| `test/test_preflight_resources.py` | `tegrastats`, thermal trip, OOM과 kernel log 경계 판정을 검사한다. |
| `test/test_preflight_runtime_boundaries.py` | observer 종료가 rclpy 내부 subscription registry를 훼손하지 않는지 검사한다. |
| `test/test_static_tf_profiles.py` | 기본·DimOS profile의 값·범위, quaternion 정규화와 unknown profile 거부를 검사한다. |
| `test/test_tf_owner_audit.py` | 단일·중복·unknown GID의 global TF owner cardinality를 검사한다. |
