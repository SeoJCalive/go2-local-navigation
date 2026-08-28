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
| `final_mount_integration.md` | 14단계 최종 장착·배선·전원·열·footprint 확인 절차를 설명한다. |
| `limited_physical_motion_validation.md` | 15단계 축별 물리 시험, 승인·StopMove·측정 record 절차를 설명한다. |
| `structured/project_manifest.yaml` | 프로젝트 범위, 현재 완료 단계, 안전 비목표와 근거 위치를 구조화한다. |
| `structured/acceptance_matrix.yaml` | 모듈 ID, 입출력, 현재 검증 수준, 근거 ID, 합격 조건, 보류 시험과 재검증 조건을 구조화한다. |
| `structured/final_mount_acceptance.yaml` | 14단계 준비 완료와 실제 장착 보류 항목을 구조화한다. |
| `structured/limited_physical_motion_acceptance.yaml` | 15단계 준비·recorder 상태와 실제 trial 보류 항목을 구조화한다. |

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
| `synthetic-verified` | 결정론적 합성 입력에서 ROS graph·action 계약을 확인했다. |
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
- 11단계 `software_fault_recovery`: `completed`
- 11단계 근거: `data/runs/fault_acceptance/stage11.json`, fault 10개 모두 PASS
- 12단계 `mapping_localization_and_nav2_shadow`: `software_replay_continuity_resolved_for_exact_canonical_bag_profile_localization_and_nav2_pending`
- 12단계 입력 근거: `data/runs/mapping_input/stage12-ingress.json`, 정지·external short 모두 PASS
- 12단계 mapping 근거: `data/runs/mapping/stage12.json`, 정지·external full 모두 PASS
- 12단계 mapping record: `go2-local-navigation-slam-mapping-replay-20260827`
- 역사적 TF continuity failed/open: `go2-local-navigation-dimos-tf-profile-ab-20260828`, 같은 120초 bag의
  `project_default`·`dimos_replay` 모두 translation·yaw step 기준 실패
- 후속 runtime: `data/runs/mapping_tf_ab/20260828_dimos_tf_ab_retry1/summary.json`,
  `toggle_confirmed=false`
- 역사적 scan projection failed/open 근거:
  `go2-local-navigation-dimos-scan-projection-ab-20260828`, 유효 beam 중앙값
  `36 → 251`과 translation·yaw 초과 횟수 감소
- scan projection runtime:
  `data/runs/mapping_scan_ab/20260828_dimos_scan_ab_round3/summary.json`, 최대
  translation·yaw step 기준 실패, `toggle_confirmed=false`
- canonical resolution: `go2-local-navigation-dimos-slam-continuity-resolution-20260828`,
  `dimos_odom_accumulated_emit3` final A/B와 two repeats 모두 passed
- canonical runtime: `data/runs/mapping_slam_fix/20260828_stable_emit3_loop_on_final_ab/summary.json`,
  candidate `0.41047936583987504 m` / `0.1800169168112875 rad`, exceedance/unaligned/regressive `0/0/0`
- canonical profile: `frame_limit=3`, `emit_every=3`, input/retry queue `64/64`,
  `min_height=-0.10`, converter queue `64`; response expansion `false`와 loop closing
  `true`는 A/B 공통 통제값, candidate-only coarse는 `0.1745`, baseline coarse는
  `0.349`이며 global launch default `0.349`/`true`/`true`는 유지
- final baseline: `raw_single`은 `0.5535118551684397 m` / `0.3678837777692796 rad`로 failed;
  candidate는 cloud `1843/1842/614`, intrinsic drop `1`, overflow/pending/regression `0/0/0`,
  command/control `0`, clean teardown, map·pose graph save/reload와 repeat 1·2를 통과
- RViz/build: Global/Map `Ok`, candidate 새 TF warning 없음, captured 25초에서 whole-view
  trembling 또는 large teleport 없음(지도 정확도 근거 아님); 해당 replay 당시
  7 packages build, 267 tests, 0 failures/errors, 3 skipped
- 현재 package 경계 clean gate: `go2_validation` 분리 후 AGX 실제 workspace
  `install/`에 8 packages build, 287 tests, 0 failures/errors, 3 skipped;
  executable `go2_validation=10`, `go2_nav2=0`; 원시 로그·JUnit·직접 QA는
  `.omo/evidence/validation-package-refactor-closeout-20260828/README.md`; software-only
  구조 검증이며 replay·live 근거 수준을 승격하지 않음
- 외부 replay 변환 근거: `data/external/dimos_go2_indoor/runs/conversion.json`
- 13단계 `software_only_freeze`: `deferred_until_stages_11_12_complete`
- 14단계 `final_mount_integration`: `preparation_completed_execution_deferred`
- 15단계 `limited_physical_motion_validation`: `preparation_completed_execution_deferred`
- recorder 근거: `go2-local-navigation-trial-recorder-readonly-qa-20260827`

10단계 경고는 30분 정지 yaw 누적 drift이며 중앙 매트릭스에서 warning으로 유지한다.
12단계에서는 PointCloud2→LaserScan 입력, replay provenance, 합성 map·BT·scenario
자산과 Domain 63 SLAM map 생성·저장·pose graph 재로딩까지 완료했다. 후속 canonical
resolution은 emit3 delivery와 profile-scoped coarse `0.1745`에서 map-correction
continuity를 통과했다. candidate cloud accounting은 `1843/1842/614`, intrinsic drop
`1`, overflow/pending/regression `0/0/0`이고 artifact 저장·재로딩과 clean teardown,
repeat도 통과했다. 이 해결은 Go2 OFF·physical execution false·command publication
false인 exact bag/profile의 replay 범위다. 앞선 TF/10-frame A/B failed/open 관찰은
조건부 superseded된 역사적 근거로 보존한다. live hardware와 physical mount, map
ground truth, saved-map localization, 전체 Nav2·NavigateToPose는 미검증이므로 다음
software 단계도 이 항목들을 별도로 판정한다.
14·15단계의 준비 완료는 실제 장착·전원·물리 command·축·StopMove 검증 완료를
의미하지 않는다. 재개 조건은 각 단계 Markdown과 YAML에서 확인한다.

## 안전 경계

이 인덱스와 과거 검증 결과는 현재 실행 승인이 아니다. 실제
`/api/sport/request`, `/lowcmd`, stand, walk, motor, navigation motion은 사용자의
최신 명시적 승인 없이는 실행하지 않는다. RealSense IMU는 현재 프로젝트 범위에서
제외한다.
