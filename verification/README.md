# 검증 진입점

이 디렉터리는 `go2_local_navigation`의 각 모듈이 현재 어디까지 검증됐고,
어떤 조건에서 무엇을 다시 확인해야 하는지 찾을 때 사용한다. 실제 측정값과 실행
상세를 다시 기록하지 않고, 패키지별 contract와 로컬 실행 record를 연결하는
반복 참조 인덱스 역할만 한다.

## 파일 구조

```text
verification/
├── README.md
├── final_mount_integration.md
├── limited_physical_motion_validation.md
└── structured/
    ├── project_manifest.yaml
    ├── acceptance_matrix.yaml
    ├── final_mount_acceptance.yaml
    └── limited_physical_motion_acceptance.yaml
```

| 파일 | 역할 |
| --- | --- |
| `README.md` | 검증 수준, 판정 상태, 읽는 순서와 갱신 기준을 설명한다. |
| `final_mount_integration.md` | 11단계 최종 장착·배선·전원·열·footprint 확인 절차를 설명한다. |
| `limited_physical_motion_validation.md` | 12단계 축별 물리 시험, 승인·StopMove·측정 record 절차를 설명한다. |
| `structured/project_manifest.yaml` | 프로젝트 범위, 현재 완료 단계, 안전 비목표와 근거 위치를 구조화한다. |
| `structured/acceptance_matrix.yaml` | 모듈 ID, 입출력, 현재 검증 수준, 근거 ID, 합격 조건, 보류 시험과 재검증 조건을 구조화한다. |
| `structured/final_mount_acceptance.yaml` | 11단계 준비 완료와 실제 장착 보류 항목을 구조화한다. |
| `structured/limited_physical_motion_acceptance.yaml` | 12단계 준비·recorder 상태와 실제 trial 보류 항목을 구조화한다. |

## 읽는 순서

1. `structured/project_manifest.yaml`에서 프로젝트 범위와 현재 단계를 확인한다.
2. `structured/acceptance_matrix.yaml`에서 `module_id`를 찾는다.
3. `current_claim`과 `attained_evidence_levels`로 현재 주장 가능한 범위를 확인한다.
4. `acceptance_checks`에서 통과·보류된 항목과 필요한 조건을 확인한다.
5. 정확한 수치나 실행 조건이 필요할 때만 `evidence_refs`가 가리키는 contract 또는
   로컬 `records/` 원문을 읽는다.

중앙 매트릭스는 관찰값의 원본이 아니다. 토픽 주기, 메시지 수, checksum, 제한값처럼
구체적인 값은 각 패키지 contract와 실행 record가 소유한다.

## 검증 수준

검증 수준은 서로 다른 근거를 덮어쓰지 않고 누적해서 기록한다.

| 수준 | 의미 |
| --- | --- |
| `implemented` | 코드·설정·launch가 존재한다. |
| `automated-tested` | 순수 계약 또는 구성 자동 테스트를 통과했다. |
| `replay-verified` | rosbag 또는 격리 입력으로 실행 경계를 확인했다. |
| `stationary-onboard-verified` | 실제 AGX와 Go2가 연결된 정지 상태에서 확인했다. |
| `final-mount-verified` | AGX를 최종 고정한 물리 구성에서 다시 확인했다. |
| `dynamic-verified` | 실제 이동 중 방향·거리·정지·센서 반응을 확인했다. |
| `accepted` | 이 모듈에 필요한 최종 조건을 모두 충족해 운영 기준으로 승인했다. |

`replay-verified`는 `stationary-onboard-verified`나 `dynamic-verified`를 대신하지
않는다. 높은 수준의 이름이 낮은 수준의 모든 시험을 자동 포함한다는 의미도 아니다.
매트릭스의 `attained_evidence_levels`와 개별 합격 항목을 함께 확인한다.

## 판정 상태

| 상태 | 의미 |
| --- | --- |
| `verified` | 적힌 범위의 주장이 직접 근거로 확인됐다. |
| `partial` | 일부 조건은 확인됐지만 같은 모듈의 필수 검증이 남아 있다. |
| `deferred` | 최종 고정이나 물리 동작처럼 아직 제공되지 않은 조건이 필요하다. |
| `unverified` | 구현 또는 실행 근거가 아직 없다. |
| `excluded` | 현재 프로젝트 범위에서 의도적으로 사용하지 않는다. |

`deferred`는 실패나 구현 불가를 의미하지 않는다. 필요한 조건과 재개 지점을
`acceptance_checks`에 반드시 함께 기록한다.

## 갱신 기준

다음 변화가 생기면 관련 모듈만 갱신한다.

- package contract의 ID·status·입출력·제한값 변경
- 새로운 replay 또는 onboard 실행 record 생성
- TF owner, frame, topic, QoS, Nav2 plugin 변경
- AGX 최종 고정, footprint 실측, 전원·네트워크 구성 변경
- motion gate 승인, 실제 축·StopMove·제동 검증 수행
- mapping, `map → odom`, 목적지 도달 결과 생성

새 실행 결과는 먼저 로컬 `records/experiments/`에 원문과 필요한 YAML projection으로
기록한다. 그 결과가 현재 주장 범위를 바꿀 때만 중앙 매트릭스의 근거와 상태를
갱신한다. 실행 로그나 수치를 중앙 매트릭스에 반복 복사하지 않는다.

## 현재 단계

- 9단계 `integrated_non_actuating_preflight`: `completed`
- 10단계 `stationary_smoke_and_soak`: `completed_with_warning`
- 10단계 근거: `go2-local-navigation-stationary-soak-20260827`
- smoke: `20260827_150959_stage10_smoke`, 47 PASS
- soak: `20260827_151053_stage10_soak`, 46 PASS·1 WARN
- 11단계 `final_mount_integration`: `preparation_completed_execution_deferred`
- 12단계 `limited_physical_motion_validation`: `preparation_completed_execution_deferred`
- recorder 근거: `go2-local-navigation-trial-recorder-readonly-qa-20260827`

10단계 경고는 30분 정지 yaw 누적 drift이며 중앙 매트릭스에서 warning으로 유지한다.
11·12단계의 준비 완료는 실제 장착·전원·물리 command·축·StopMove 검증 완료를
의미하지 않는다. 재개 조건은 각 단계 Markdown과 YAML에서 확인한다.

## 안전 경계

이 인덱스와 과거 검증 결과는 현재 실행 승인이 아니다. 실제
`/api/sport/request`, `/lowcmd`, stand, walk, motor, navigation motion은 사용자의
최신 명시적 승인 없이는 실행하지 않는다. RealSense IMU는 현재 프로젝트 범위에서
제외한다.
