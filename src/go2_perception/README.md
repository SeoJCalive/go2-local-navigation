# `go2_perception`

이 패키지는 Go2 로컬 내비게이션에서 정지 상태 LiDAR를 읽어 base-frame
obstacle candidate PointCloud2로 보고하는 읽기 전용 경계다. candidate는 최종
장애물 분류도 free-space 증명도 아니다. 별도의 mapping ingress는 malformed·empty·
NaN-only·stale cloud를 차단한 뒤 공식 `pointcloud_to_laserscan` node에 전달한다.

## 입력과 출력 경계

- 입력: `/utlidar/cloud` (`sensor_msgs/msg/PointCloud2`), `RELIABLE`,
  `KEEP_LAST(1)`, `VOLATILE`, frame `utlidar_lidar`
- TF: node가 message timestamp로 기존 static TF tree를 lookup해 `base`로
  XYZ를 변환한다. perception은 extrinsic 수치를 복제하거나 추정하지 않는다.
- 필터: 변환 뒤 유한한 point만 planar range `[0.25, 5.0]` m 및 base-frame z
  `[-0.25, 1.0]` m의 포함 경계로 남긴다.
- 출력: `/perception/obstacle_candidates` (`sensor_msgs/msg/PointCloud2`),
  input header timestamp를 유지하고 frame은 `base`다.
- mapping 출력: `/go2_mapping/cloud_validated`를 거쳐 `/scan`
  (`sensor_msgs/msg/LaserScan`, frame `base`)으로 변환한다.
- 외부 replay 누적 profile은 validated cloud의 compact XYZ를 cloud timestamp의
  `odom` frame으로 바꿔 `/go2_mapping/cloud_accumulated`로 결합한다. `dimos_odom_accumulated_emit3`는 frame 3·emit 3, `min_height=-0.10`, queue 64의 DimOS 외부 replay 전용 engineering candidate이며 physical suitability는 unverified다.
- mapping launch는 `sensor_tf_profile`을 static TF launch까지 전달한다. 기본값은
  `project_default`이고 `dimos_replay`는 해당 외부 bag에서만 명시한다.
- mapping launch의 기본 `execution_mode`는 `onboard`, scan profile은 `raw_single`이다.
  `dimos_external_replay_only` 범위의 누적 profile은 `external_replay` mode에서만
  선택되며, 다른 조합은 node 시작 전에 typed error로 거부된다.
- 잘못된 PointCloud2 layout은 candidate callback과 mapping gate 양쪽에서 publish 없이
  경고로 남기며 node를 종료시키지 않는다.
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
│   ├── mapping_scan.yaml
│   └── perception_contract.yaml
├── launch/
│   └── go2_mapping_scan.launch.py
├── go2_perception/
│   ├── __init__.py
│   ├── mapping_cloud_accumulator.py
│   ├── mapping_cloud_accumulator_node.py
│   ├── mapping_cloud_contract.py
│   ├── mapping_cloud_gate_node.py
│   ├── mapping_scan_profiles.py
│   ├── obstacle_candidate_node.py
│   └── perception_contract.py
└── test/
    ├── test_mapping_cloud_accumulator.py
    ├── test_mapping_cloud_contract.py
    ├── test_mapping_scan_configuration.py
    ├── test_obstacle_candidate_faults.py
    ├── test_obstacle_candidate_shutdown.py
    └── test_perception_contract.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `README.md` | stationary perception 입력·출력, candidate 의미와 안전 경계를 설명한다. |
| `package.xml` | ROS 2 metadata와 perception에 필요한 message·Python 의존성을 선언한다. |
| `setup.py` | ament_python metadata, config·launch 설치와 obstacle·mapping gate·누적 executable을 등록한다. |
| `setup.cfg` | 개발·설치 시 script 경로를 지정한다. |
| `resource/go2_perception` | ament index가 패키지를 찾기 위한 빈 marker 파일이다. |
| `config/mapping_scan.yaml` | validated cloud 입력, LaserScan frame·각도·높이·range와 domain 62 provenance 및 replay-only/unverified DimOS emit3 profile을 정의한다. |
| `config/perception_contract.yaml` | project-observed 입력, project target, filter와 candidate 상태를 구조화한다. |
| `launch/go2_mapping_scan.launch.py` | 선택된 static sensor TF profile, mapping cloud gate와 공식 PointCloud2→LaserScan 변환기를 시작한다. |
| `go2_perception/__init__.py` | Python package 경계와 candidate 의미를 설명한다. |
| `go2_perception/mapping_cloud_accumulator.py` | compact XYZ cloud의 동일 frame·layout을 확인하고 고정 길이 sliding window를 결합한다. |
| `go2_perception/mapping_cloud_accumulator_node.py` | cloud stamp의 odom TF를 적용해 외부 replay용 누적 cloud를 publish한다. |
| `go2_perception/mapping_cloud_contract.py` | layout·빈 입력·NaN-only·timestamp·stale cloud의 순수 차단 규칙이다. |
| `go2_perception/mapping_cloud_gate_node.py` | 유효한 raw cloud만 `/go2_mapping/cloud_validated`로 전달한다. |
| `go2_perception/mapping_scan_profiles.py` | raw와 DimOS 외부 replay 전용 누적 profile을 YAML에서 파싱한다. |
| `go2_perception/obstacle_candidate_node.py` | TF lookup, PointCloud2 변환·필터·candidate publish만 수행하는 ROS node다. |
| `go2_perception/perception_contract.py` | ROS 의존성 없는 point 변환과 candidate filter 계약이다. |
| `test/test_mapping_cloud_accumulator.py` | window eviction, padded mixed cloud compact XYZ 변환과 profile 분리를 검사한다. |
| `test/test_mapping_cloud_contract.py` | mapping cloud 차단 사유와 valid recovery 계약을 검사한다. |
| `test/test_mapping_scan_configuration.py` | 공식 converter 사용, remap, frame과 candidate parameter를 검사한다. |
| `test/test_obstacle_candidate_faults.py` | malformed cloud가 candidate node를 종료시키지 않고 차단되는지 검사한다. |
| `test/test_obstacle_candidate_shutdown.py` | ROS context 종료 뒤 중복 shutdown·logger 접근이 없는지 검사한다. |
| `test/test_perception_contract.py` | contract, quaternion geometry, range·z·finite filter 순수 unit test다. |

bringup의 `go2_stationary_perception.launch.py`는 기존 static TF launch와 이
node만 시작한다. stationary bag E2E에서 raw cloud `300`개와 output `300`개의
timestamp가 정확히 일치했고, output은 모두 frame `base`, fields `x/y/z`, finite
point와 설정 경계 안의 값으로 확인됐다. 상세 수치는
`records/experiments/go2_local_navigation_stationary_bag_perception_20260826.md`에
보존한다.

2026-08-28 DimOS short bag의 TF profile A/B에서는 source-aligned profile이
`map → odom` jump를 줄였지만 연속성 기준을 통과하지 못했다. 후속 cloud 19개 표본은
현재 height filter 뒤 0.5도 유효 beam 중앙값이 `37`이었고 filter를 더 좁히면
`10`으로 감소했다. 따라서 height 범위를 임의로 축소하지 않고, 짧은 시간 cloud
누적 또는 다른 2D projection을 별도 A/B할 후보로 유지한다. 이 표본은 scan 품질의
최종 합격 근거나 원인 확정이 아니다.

후속 120초 scan projection A/B에서 10-frame 누적은 유효 beam 중앙값을
`36 → 251`로 높이고 translation·yaw 기준 초과 횟수를 `487 → 94`, `49 → 4`로
줄였다. 그러나 최대 step은 `8.221949 m`·`0.358414 rad`로 기준을 계속 초과했다.
또한 future odometry TF가 아직 도착하지 않은 cloud `571`개가 차단됐다. 이 profile은
부분 개선이 확인된 외부 replay 실험 후보이며 실물 기본 경로가 아니다.
