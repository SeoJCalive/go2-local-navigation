# go2_state_estimation

이 패키지는 AGX의 `/utlidar/robot_odom` source 계약을 관찰하고, 프로젝트에서
수용한 frame mapping이 적용된 경우 `/odom`과 `odom → base` dynamic TF를 제공한다.
source probe는 계속 읽기 전용이며, adapter는 유효한 source sample만 출력한다.

현재 source child frame은 `base_link`로 관찰됐고 프로젝트 canonical target frame은
`base`다. 이 패키지는 `base_link → base`를 프로젝트 통합 mapping으로 수용한다.
이는 공식 onboard frame 정의나 독립 물리 calibration의 확정이 아니며, adapter는
source child가 정확히 `base_link`일 때만 동작한다.

## 폴더 및 파일 구조

```text
go2_state_estimation/
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── go2_state_estimation
├── config/
│   └── odometry_contract.yaml
├── go2_state_estimation/
│   ├── __init__.py
│   ├── continuity_profiles.py
│   ├── odometry_adapter_node.py
│   ├── odometry_contract.py
│   └── odometry_probe_node.py
└── test/
    ├── test_continuity_profiles.py
    ├── test_odometry_adapter_shutdown.py
    └── test_odometry_contract.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `README.md` | source 관찰, frame mapping, continuity gate와 미확인 정확도 경계를 설명한다. |
| `package.xml` | ament_python과 Odometry·TF runtime 의존성을 선언한다. |
| `setup.py` | config 설치와 probe·adapter console entry point를 등록한다. |
| `setup.cfg` | Python 실행 파일 설치 경로를 지정한다. |
| `resource/go2_state_estimation` | ament index가 패키지를 찾는 marker다. |
| `config/odometry_contract.yaml` | source·project frame, QoS, continuity 후보값과 unresolved warning을 구조화한다. |
| `go2_state_estimation/__init__.py` | odometry 관찰과 adapter의 Python package 경계를 설명한다. |
| `go2_state_estimation/continuity_profiles.py` | YAML의 `onboard_observe`·`replay_enforce` profile과 동일 후보 제한값을 typed 불변 값으로 파싱한다. |
| `go2_state_estimation/odometry_adapter_node.py` | 유효한 source를 `/odom`과 `odom → base`로 전달하고 선택된 profile에 따라 continuity 위반을 관찰하거나 차단한다. |
| `go2_state_estimation/odometry_contract.py` | frame·timestamp·finite value와 regression·jump·loss·recovery 순수 규칙이다. |
| `go2_state_estimation/odometry_probe_node.py` | source를 변경하지 않고 frame·covariance·invalid sample을 logger로 관찰한다. |
| `test/test_continuity_profiles.py` | 두 profile이 같은 evaluator·후보값을 사용하면서 publish 결정만 달리하는지 검사한다. |
| `test/test_odometry_adapter_shutdown.py` | inactive ROS context의 종료 경계를 검사한다. |
| `test/test_odometry_contract.py` | source 계약과 enforce profile의 suppression·2-sample recovery를 검사한다. |

## 실행 경계

- 입력: `/utlidar/robot_odom` (`nav_msgs/msg/Odometry`)만 구독한다.
- QoS: `RELIABLE`, `KEEP_LAST(1)`, `VOLATILE`.
- `odometry_probe`: logger만 사용하며 source child frame과 covariance 상태를 기록한다.
- `odometry_adapter`: `/odom`에 child `base`인 `nav_msgs/msg/Odometry`를 publish하고,
  같은 pose·timestamp로 dynamic `odom → base` TF를 publish한다.
- adapter는 `/cmd_vel` publish, service call, motion을 수행하지 않으며, 잘못된
  source frame·timestamp·수치 sample은 출력하지 않는다.
- continuity 계산과 recovery 상태 기계는 하나다. 일반 실행의 `onboard_observe`는
  source 계약이 유효한 sample의 jump·stale·regression을 기록하면서 출력을 유지하고,
  외부 replay·fault의 `replay_enforce`는 같은 위반을 차단한다. profile 변경은 launch
  parameter로 수행하므로 두 상황을 오갈 때 source 파일을 수정하지 않는다.
- timestamp gap `0.5 s`, 단일 translation delta `0.5 m`, yaw delta `0.5 rad`,
  연속 valid sample `2`개는 두 profile이 공유하는 engineering candidate다. 실제
  이동 안전 한계나 odometry 정확도 합격값이 아니며, map-correction continuity의
  별도 `0.5 m`·`0.2 rad` 판정과도 같은 기준이 아니다.
- executor의 message 변환 `RuntimeError`는 ROS context가 이미 inactive인 종료
  경계에서만 정상 종료로 처리한다. active context의 같은 오류는 숨기지 않고 다시
  발생시킨다.

이 README와 `config/odometry_contract.yaml`은 source 관찰과 project adapter의
실행 계약을 설명한다. `odom → base` owner는 `odometry_adapter_node` 하나로
정한다. 공식 reference와 project mapping의 구분은 결정 record에 보존한다.

2026-08-23 AGX에서 read-only probe는 `3851`개 sample을 받아 invalid `0`으로
확인했고, adapter runtime은 `10634`개를 받아 `10633`개를 publish하고 `0`개를
reject했다. `/odom` publisher와 dynamic `odom → base` TF를 확인한 뒤 두 launch를
teardown했으며, command topic baseline은 변하지 않았다. all-zero covariance는
source 값을 그대로 전달했고 수치 보정하지 않았다.

30분 정지 soak의 yaw 누적 drift `0.248012 rad`는 해결되지 않은 warning으로
유지한다. continuity gate의 단일-sample yaw 한계와 장시간 누적 drift는 서로 다른
판정이며, 후자를 통과로 해석하지 않는다.

2026-08-27 Nav2 preview 종료 재검증에서 executor shutdown 경계 회귀 테스트를
추가했고, 실제 launch 종료 때 adapter가 clean exit하는 것을 확인했다.

상세 결과는
`records/experiments/go2_local_navigation_odom_adapter_20260823.md`와 같은 stem의
YAML projection에서 확인한다.
