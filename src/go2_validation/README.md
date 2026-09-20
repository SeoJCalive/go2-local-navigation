# `go2_validation`

`go2_validation`은 fault, 합성 Nav2 shadow, Domain 0 live no-goal observer와
통합 preflight를 담당하는 software-only 검증 package다. 실제
command publish, motion 또는 physical execution을 소유하지 않는다.

mapping/localization bag replay의 취득·변환·player·runner·A/B·sweep 계층은
제거됐다. 실제 Go2가 사용하는 mapping/localization launch 자산은 `go2_nav2`에 남아 있다.

## 설치되는 실행 파일

- `integrated_preflight`
- `navigation_runtime_preflight`
- `fault_fixture`
- `fault_acceptance`
- `live_navigation_acceptance`
- `shadow_fixture`
- `nav2_shadow_acceptance`

## 주요 파일

| 파일 | 역할 |
| --- | --- |
| `config/execution_modes.yaml` | fault·synthetic·live mode의 domain, clock, TF owner 경계를 정의한다. |
| `config/shadow_scenarios.yaml` | 합성 navigation 시나리오와 기대 terminal 상태를 정의한다. |
| `launch/go2_fault_acceptance.launch.py` | 합성 fault 입력과 비동작 downstream을 조합한다. |
| `launch/go2_integrated_preflight.launch.py` | controller preview와 observer를 단일 종료 경계로 조합한다. |
| `go2_validation/fault_*` | fault fixture, 관찰, 판정과 실행 lifecycle을 담당한다. |
| `go2_validation/live_navigation_*` | 실제 Go2 stream을 no-goal·비동작으로 관찰한다. |
| `go2_validation/mapping_artifacts.py` | live saved-map YAML과 image의 identity를 계산한다. |
| `go2_validation/mapping_runtime_graph.py` | live·shadow observer가 공유하는 QoS와 graph helper다. |
| `go2_validation/offline_process.py` | 격리 launch process의 bounded wait와 teardown을 담당한다. |
| `go2_validation/shadow_*` | 합성 Nav2 action 시나리오와 안전 판정을 담당한다. |

## 실행 경계

통합 preflight는 다음처럼 실행한다.

```bash
ros2 run go2_validation integrated_preflight --ros-args -p duration_sec:=30
```

`live_navigation_acceptance`는 실제 입력과 저장 지도를 사용하지만 action goal을
보내지 않는다.
