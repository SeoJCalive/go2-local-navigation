# `go2_validation`

`go2_validation`은 fault, external replay, mapping, 통합 preflight의
software-only 검증 도구 package다. Nav2 runtime asset을 `go2_nav2`에서 읽어
조합할 수 있지만 production navigation runtime component가 아니며, 실제 command
publish·motion·physical execution을 소유하지 않는다.

기존 executable 이름은 유지한다. 예를 들어 통합 preflight는 다음 package owner로
실행한다.

```bash
ros2 run go2_validation integrated_preflight --ros-args -p duration_sec:=30
```

## 폴더 및 파일 구조

```text
go2_validation/
├── README.md
├── package.xml
├── resource/
│   └── go2_validation
├── setup.cfg
├── setup.py
├── config/
│   ├── execution_modes.yaml
│   ├── external_replay_sources.yaml
│   └── shadow_scenarios.yaml
├── launch/
│   ├── go2_fault_acceptance.launch.py
│   └── go2_integrated_preflight.launch.py
├── go2_validation/
│   ├── __init__.py
│   ├── external_replay_acquisition.py
│   ├── external_replay_acquisition_runner.py
│   ├── external_replay_contract.py
│   ├── external_replay_conversion.py
│   ├── external_replay_conversion_result.py
│   ├── external_replay_conversion_runner.py
│   ├── external_replay_converter.py
│   ├── external_replay_download.py
│   ├── external_replay_manifest.py
│   ├── external_replay_rosbag.py
│   ├── external_replay_scan.py
│   ├── external_replay_window.py
│   ├── fault_acceptance_runner.py
│   ├── fault_acceptance_runtime.py
│   ├── fault_fixture_model.py
│   ├── fault_fixture_node.py
│   ├── fault_runtime_capture.py
│   ├── fault_runtime_execution.py
│   ├── fault_runtime_observer.py
│   ├── mapping_acceptance.py
│   ├── mapping_acceptance_runner.py
│   ├── mapping_artifacts.py
│   ├── mapping_cloud_accounting.py
│   ├── mapping_command_builders.py
│   ├── mapping_coarse_search_sweep_runner.py
│   ├── mapping_input_acceptance_runner.py
│   ├── mapping_input_capture.py
│   ├── mapping_input_execution.py
│   ├── mapping_input_observer.py
│   ├── mapping_input_runtime.py
│   ├── mapping_player_services.py
│   ├── mapping_pose_continuity.py
│   ├── mapping_runtime_data.py
│   ├── mapping_runtime_execution.py
│   ├── mapping_runtime_graph.py
│   ├── mapping_runtime_observer.py
│   ├── mapping_scan_profile_ab_runner.py
│   ├── mapping_scan_quality.py
│   ├── mapping_slam_services.py
│   ├── mapping_tf_continuity.py
│   ├── mapping_tf_profile_ab_input.py
│   ├── mapping_tf_profile_ab_runner.py
│   ├── offline_process.py
│   ├── preflight_runner_configuration.py
│   ├── preflight_runner_node.py
│   ├── runtime_preflight.py
│   └── typing_compat.py
└── test/
    ├── test_external_replay_acquisition.py
    ├── test_external_replay_converter.py
    ├── test_fault_acceptance.py
    ├── test_fault_fixture.py
    ├── test_fault_runtime_capture.py
    ├── test_mapping_acceptance.py
    ├── test_mapping_cloud_accounting.py
    ├── test_mapping_input_acceptance.py
    ├── test_mapping_input_capture.py
    ├── test_mapping_player_startup.py
    ├── test_mapping_pose_continuity.py
    ├── test_mapping_runtime_graph.py
    ├── test_mapping_runtime_teardown.py
    ├── test_mapping_slam_services.py
    ├── test_mapping_stable_emit3_profile.py
    ├── test_mapping_tf_profile_ab.py
    ├── test_preflight_package_ownership.py
    ├── test_validation_package_boundary.py
    ├── test_verification_stage_alignment.py
    └── test_wave1_contracts.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `README.md` | software-only validation tool의 범위와 file ownership을 설명한다. |
| `package.xml` | validation runner가 사용하는 ROS 2 metadata와 의존성을 선언한다. |
| `resource/go2_validation` | ament index package marker다. |
| `setup.cfg` | ament Python script 설치 경로를 지정한다. |
| `setup.py` | validation config·launch를 설치하고 11개 validation executable을 이 package에만 등록한다. |
| `config/execution_modes.yaml` | domain, clock owner, global TF owner, sim time과 loopback을 software validation mode별로 정의한다. |
| `config/external_replay_sources.yaml` | pinned DimOS source의 custody, hash·size 제한, dataset 출처와 센서 identity 상태, canonical channel과 replay-only profile provenance를 정의한다. |
| `config/shadow_scenarios.yaml` | synthetic navigation 후보의 map·start/goal·terminal expectation을 정의한다. |
| `launch/go2_fault_acceptance.launch.py` | Domain 61 synthetic fault fixture와 non-actuating derived-output owner를 조합한다. `execution_mode`와 `continuity_profile`을 선언해 mapping scan과 odometry adapter에 각각 전달한다. |
| `launch/go2_integrated_preflight.launch.py` | `go2_nav2` controller preview와 bringup observer를 단일 shutdown 경계로 조합한다. |
| `go2_validation/__init__.py` | validation orchestration만 소유하고 production navigation runtime이 아님을 설명한다. |
| `go2_validation/external_replay_acquisition.py` | pinned archive의 bounded download/extract, hash·size·path 경계와 atomic promotion을 처리한다. |
| `go2_validation/external_replay_acquisition_runner.py` | acquisition spec을 실행하고 custody 결과 JSON을 기록한다. |
| `go2_validation/external_replay_contract.py` | external replay channel·schema·count·semantic equality의 순수 계약을 정의한다. |
| `go2_validation/external_replay_conversion.py` | verified raw DDS MCAP을 canonical short/full rosbag fixture로 변환한다. |
| `go2_validation/external_replay_conversion_result.py` | conversion inventory와 passed/deferred/conflict 결과 schema를 정의한다. |
| `go2_validation/external_replay_conversion_runner.py` | acquisition 결과를 읽어 conversion을 실행하고 result JSON을 기록한다. |
| `go2_validation/external_replay_converter.py` | CDR canonicalization, deterministic output과 output-tree checksum 경계를 처리한다. |
| `go2_validation/external_replay_download.py` | pinned artifact의 bounded curl download와 HTTP/파일 검증을 처리한다. |
| `go2_validation/external_replay_manifest.py` | external replay YAML을 acquisition·conversion의 불변 spec으로 파싱한다. |
| `go2_validation/external_replay_rosbag.py` | mixed MCAP source reader와 Humble canonical rosbag writer를 연결한다. |
| `go2_validation/external_replay_scan.py` | raw source의 CDR, 시간 구간, frame과 planar odometry를 조사한다. |
| `go2_validation/external_replay_window.py` | 120초 dynamic window 후보와 deterministic 선택 규칙을 계산한다. |
| `go2_validation/fault_acceptance_runner.py` | Stage 11 fault acceptance의 순수 oracle과 좁은 entry-point 경계를 제공한다. |
| `go2_validation/fault_acceptance_runtime.py` | Domain 61 fault matrix를 순차 실행하고 report JSON을 기록한다. |
| `go2_validation/fault_fixture_model.py` | ROS-independent fault kind, timeline, output count와 expected exit contract를 정의한다. |
| `go2_validation/fault_fixture_node.py` | isolated domain에서 synthetic cloud·odometry·clock fault input을 publish한다. |
| `go2_validation/fault_runtime_capture.py` | fixture marker와 실제 downstream stamp를 결합한 순수 capture를 만든다. |
| `go2_validation/fault_runtime_execution.py` | 한 fault launch attempt의 child lifecycle, observer, teardown 결과를 소유한다. |
| `go2_validation/fault_runtime_observer.py` | Domain 61 event·derived output·TF·command graph를 읽기 전용으로 관찰한다. |
| `go2_validation/mapping_acceptance.py` | mapping stream, ownership, artifact와 teardown observation을 passed/failed verdict로 판정한다. |
| `go2_validation/mapping_acceptance_runner.py` | stationary·external-full Domain 63 validation을 순차 실행하고 summary JSON을 기록한다. |
| `go2_validation/mapping_artifacts.py` | occupancy map·image·pose graph 저장과 reload artifact 경계를 검증한다. |
| `go2_validation/mapping_cloud_accounting.py` | accumulated mapping cloud node의 terminal accounting marker를 파싱한다. |
| `go2_validation/mapping_command_builders.py` | Domain 63 launch/player의 shell-free argv와 profile-specific parameter를 조립한다. |
| `go2_validation/mapping_coarse_search_sweep_runner.py` | canonical DimOS short bag에서 emit3 후보를 순차 비교하고 후보별 artifact·연속성 결과를 기록한다. 기본은 7개이며 `angle_offsets_rad` parameter로 lower-band 후보를 주입할 수 있다. |
| `go2_validation/mapping_input_acceptance_runner.py` | Domain 62 mapping input observation의 순수 verdict와 sequential variant spec을 정의한다. |
| `go2_validation/mapping_input_capture.py` | scan·odom·graph 표본을 mapping-input acceptance observation으로 투영한다. |
| `go2_validation/mapping_input_execution.py` | Domain 62 variant의 mapping scan/odometry launch와 bounded rosbag player lifecycle을 소유한다. mapping scan에는 `execution_mode`만, external odometry에는 `continuity_profile=replay_enforce`를 전달한다. |
| `go2_validation/mapping_input_observer.py` | Domain 62 scan·odom·clock·global TF·command graph를 관찰한다. |
| `go2_validation/mapping_input_runtime.py` | stationary와 external-short ingress를 순차 실행하고 JSON을 기록한다. |
| `go2_validation/mapping_player_services.py` | paused rosbag player의 graph readiness와 Resume service handshake를 처리한다. |
| `go2_validation/mapping_pose_continuity.py` | 공통 odom pose에서 map-to-odom correction step을 bounded 통계로 측정한다. |
| `go2_validation/mapping_runtime_data.py` | rosbag metadata, custody와 checksum을 immutable runtime data로 파싱한다. |
| `go2_validation/mapping_runtime_execution.py` | Domain 63 mapping variant의 launch, player, save/reload와 teardown lifecycle을 소유한다. |
| `go2_validation/mapping_runtime_graph.py` | mapping observer가 쓰는 QoS와 ROS graph path helper를 정의한다. |
| `go2_validation/mapping_runtime_observer.py` | Domain 63 stream·clock·TF·command graph를 읽기 전용으로 관찰한다. |
| `go2_validation/mapping_scan_profile_ab_runner.py` | 같은 DimOS TF에서 raw와 odometry-accumulated scan profile A/B를 실행하고 기록한다. |
| `go2_validation/mapping_scan_quality.py` | LaserScan payload 없이 valid beam 분포만 누적한다. |
| `go2_validation/mapping_slam_services.py` | SLAM Toolbox save, serialize, deserialize service의 readiness와 결과를 처리한다. |
| `go2_validation/mapping_tf_continuity.py` | map-to-odom transform의 translation·yaw step bounded 통계를 누적한다. |
| `go2_validation/mapping_tf_profile_ab_input.py` | TF profile A/B의 external replay input·profile·provenance spec을 정의한다. |
| `go2_validation/mapping_tf_profile_ab_runner.py` | 동일 external short bag의 TF profile A/B를 순차 실행하고 toggle verdict를 기록한다. |
| `go2_validation/offline_process.py` | isolated child process의 bounded spin과 parent-first stop을 제공한다. |
| `go2_validation/preflight_runner_configuration.py` | integrated preflight duration, label, output path를 runner configuration으로 파싱한다. |
| `go2_validation/preflight_runner_node.py` | host lifecycle, resource capture, observer launch와 final preflight result를 조합한다. |
| `go2_validation/runtime_preflight.py` | validation mode의 domain·clock·sim-time·network observation을 fail-fast 판정한다. |
| `go2_validation/typing_compat.py` | Python compatibility를 위한 exhaustive variant helper를 제공한다. |
| `test/test_external_replay_acquisition.py` | archive traversal, hash, size, free-space, cleanup과 custody 경계를 검사한다. |
| `test/test_external_replay_converter.py` | channel, CDR semantic, window, determinism, QoS와 output cap을 검사한다. |
| `test/test_fault_acceptance.py` | fault suppression, recovery, safety oracle와 fault launch argv를 검사한다. |
| `test/test_fault_fixture.py` | deterministic fixture timeline, NAN serialization과 odometry recovery ordering을 검사한다. |
| `test/test_fault_runtime_capture.py` | fixture marker와 downstream capture projection을 검사한다. |
| `test/test_mapping_acceptance.py` | mapping verdict, ownership, artifact, accounting과 runner boundary를 검사한다. |
| `test/test_mapping_cloud_accounting.py` | accumulated cloud terminal accounting marker와 profile requirement를 검사한다. |
| `test/test_mapping_input_acceptance.py` | Domain 62 ingress verdict, bag argv와 mode/profile forwarding을 검사한다. |
| `test/test_mapping_input_capture.py` | scan timestamp·rate·range와 odometry overlap projection을 검사한다. |
| `test/test_mapping_player_startup.py` | paused player discovery·Resume ordering과 typed failure를 검사한다. |
| `test/test_mapping_pose_continuity.py` | common odom pose 기반 correction continuity와 timeout alignment를 검사한다. |
| `test/test_mapping_runtime_graph.py` | graph path와 global TF owner projection, QoS helper를 검사한다. |
| `test/test_mapping_runtime_teardown.py` | launch 종료 뒤 graph settle과 residual teardown boundary를 검사한다. |
| `test/test_mapping_slam_services.py` | Humble SLAM mapping save·serialize·deserialize service contract를 검사한다. |
| `test/test_mapping_stable_emit3_profile.py` | replay-only stable emit3 profile의 accounting·continuity 선택을 검사한다. |
| `test/test_mapping_tf_profile_ab.py` | TF profile A/B, map correction continuity, scan profile과 argv selection을 검사한다. |
| `test/test_preflight_package_ownership.py` | integrated preflight owner가 `go2_validation`이고 package cycle이 없는지 검사한다. |
| `test/test_validation_package_boundary.py` | runtime-only `go2_nav2`와 validation-only `go2_validation`의 파일·executable·argv 경계를 검사한다. |
| `test/test_verification_stage_alignment.py` | software stage와 physical stage 경계가 혼합되지 않는지 검사한다. |
| `test/test_wave1_contracts.py` | execution mode, fault, mapping, odometry와 TF owner의 공통 contract를 검사한다. |

## 실행 경계

`mapping_coarse_search_sweep`는 기본적으로 기존 7개 후보를 사용한다. 특정 replay
범위만 비교할 때는 `angle_offsets_rad`에 comma-separated radian 목록을 전달한다.
예를 들어 0~4° lower-band 시험은 다음과 같이 실행한다.

```bash
ros2 run go2_validation mapping_coarse_search_sweep --ros-args \
  -p run_label:=lower_search_0to4deg \
  -p angle_offsets_rad:="0.0,0.01745,0.03490,0.05236,0.06980"
```

이 parameter는 validation runner의 후보 목록만 바꾸며, onboard 기본 profile이나
production navigation launch의 coarse search 기본값을 바꾸지 않는다.

이 package의 runner는 software-only observation, local artifact custody와 process
lifecycle을 검증한다. `offline_fault`는 `execution_modes.yaml`의 runtime domain
ID일 뿐 profile-loader execution mode가 아니다. 일반 onboard 경로는
`execution_mode=onboard`와 `continuity_profile=onboard_observe`를 유지하며,
replay 전용 profile은 해당 external replay caller가 명시한다.

`external replay`는 외부 출처 dataset을 뜻하며 외장 LiDAR를 뜻하지 않는다.
현재 canonical DimOS source의 센서 계열은 Go2 built-in L1 ULIDAR로 강하게
추론하지만 hardware manifest가 없어 `unverified`다. 함께 보관된 Mid-360·Point-LIO
DB는 canonical short·full 변환 계보에서 제외한다.

external replay acquisition·conversion과 해당 MCAP test는
`requirements/external-replay.txt`의 pinned Python dependency를 사용한다. AGX의
project-local 설치는 `.python_deps`에 있으며, 이 도구를 실행하거나 CI-equivalent
test를 수행할 때만 해당 경로를 `PYTHONPATH`에 명시한다. 이 dependency 경로는
일반 onboard runtime wrapper나 profile 기본값에 추가하지 않는다.
