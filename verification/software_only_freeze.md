# 13단계: software-only 동결

이 문서는 Go2를 움직일 수 없는 상태에서 12단계까지 구현·실행한 software 경계를
반복 참조할 기준점으로 보존한다. 실행 수치와 경로는 구조화 YAML에서 검색하고,
여기서는 무엇이 확인됐고 무엇을 주장할 수 없는지 설명한다.

## 판정

- 12단계 `mapping_localization_and_nav2_shadow`: `completed_software_only`
- 13단계 `software_only_freeze`: `completed_software_only_freeze`
- 다음 실행 단계: 14단계 `final_mount_integration`
- Go2 전원: OFF
- 물리 동작·command publication: 없음
- Domain 0 live shadow: `deferred`
- 지도 정확도·localization 정확도·실제 목적지 도달: `unverified`

12단계 완료는 Domain 63 mapping artifact 경계, Domain 64 saved-map localization
연결성과 Domain 65 합성 Nav2 action 계약을 software-only 조건에서 확인했다는 뜻이다.
외부 replay의 occupancy 품질 문제나 최종 장착·실제 주행이 해결됐다는 뜻은 아니다.

## 실행 조건

| 항목 | 값 |
| --- | --- |
| target | `go2_agx` |
| project | `/home/bi-agx1/go2_projects/projects/go2_local_navigation` |
| ROS | ROS 2 Humble, `rmw_cyclonedds_cpp` |
| 격리 | CycloneDDS interface `lo`, multicast 비활성 |
| localization | Domain `64`, rosbag player 단독 `/clock`, AMCL 단독 `map → odom` |
| Nav2 shadow | Domain `65`, fixture 단독 `/clock`·`map → odom`·`odom → base` |
| participant 탐색 | `ParticipantIndex=auto`, `MaxAutoParticipantIndex=120` |
| 물리 command | `/api/sport/request`, `/lowcmd`, `/cmd_vel` publisher 0 |

재현에 사용한 CycloneDDS URI는 다음과 같다. participant 상한은 기본 loopback URI에서
동시 participant를 찾지 못한 Domain 64 실행을 원인 분리한 뒤 추가한 실행 조건이다.
production source나 onboard wrapper 기본값은 바꾸지 않았다.

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="lo" priority="default" multicast="false" />
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

## Domain 64 saved-map localization

최종 결과는
`data/runs/localization/stage13-freeze-domain64-max120/result.json`이며 `passed`다.

| 관찰 | 결과 |
| --- | ---: |
| `/scan` | 300 |
| `/odom` | 2923 |
| `/map` | 1, frame `map`, cell 존재 |
| `/amcl_pose` | 1, finite 1 |
| lifecycle | `amcl=active`, `map_server=active` |
| global edge·owner | `map → odom`, `/amcl` 단독 |
| player·launch exit | `0 / 0` |
| command·control | `0 / 0` |
| residual node·process | `0 / 0` |
| teardown clock·global owner | `0 / 0` |

지도 SHA-256은
`f023d6d53c9818b3926a930f406263349a67a807f89c2ddf5dd7730f57ad1333`,
stationary bag tree SHA-256은
`c5e3ccef43ec754af364d06ddb09f0ab591fd360445d20910289d6134c741fcd`다.
결과 JSON SHA-256은
`722c09b3785e3305700921b73ccb55c2de8e587b300e5cf596d319185b0d106a`다.

기본 loopback URI로 실행한 첫 결과
`data/runs/localization/stage13-freeze-domain64/result.json`은
`mapping_player_readiness_failed`였다. launch 로그에서 Map Server와 AMCL은 active였지만
`mapping_cloud_gate`와 rosbag player가 `Failed to find a free participant index for
domain 64`로 초기화에 실패했다. 동일 source·map·bag에서 participant 탐색 상한만
추가한 뒤 통과했으므로 source 결함이나 AMCL lifecycle 실패로 분류하지 않는다.

이 결과는 저장 지도와 AMCL을 연결하고 단일 owner·finite output·종료 경계를
확인한다. 실제 위치와 비교한 ground truth가 없으므로 localization 정확도 합격은
아니다.

## Domain 65 Nav2 shadow

최종 결과는
`data/runs/nav2_shadow/stage13-freeze-domain65-max120/summary.json`이며 여섯
시나리오가 모두 `passed`다.

| scenario | action terminal | path | inert candidate | 판정 |
| --- | --- | ---: | ---: | --- |
| `success` | `SUCCEEDED` | 27 | 269 | passed |
| `cancel` | `CANCELED` | 1 | 1 | passed |
| `blocked_goal` | `ABORTED` | 0 | 0 | passed |
| `outside_map_goal` | `ABORTED` | 0 | 0 | passed |
| `planner_failure` | `ABORTED` | 0 | 0 | passed |
| `no_progress` | `ABORTED` | 8 | 62 | passed |

각 시나리오에서 Map Server, Planner, Controller, Behavior Server와 BT Navigator가
active였고 global/local costmap이 발행됐다. fixture만 합성 시간과 두 TF edge를
소유했다. velocity는 `/go2_nav2/shadow_cmd_vel`에서만 소비됐으며 physical command,
Go2 control node, Unitree node는 0이었다. 모든 fixture·launch exit는 `0/0`, 종료 뒤
잔류 node·process·clock·TF owner도 0이었다. summary SHA-256은
`6f312dbe9a586bfd84da83fe6d54a21d77486a7da364b79bfab629d69f1d19f7`다.

이 합성 fixture는 Nav2 action, lifecycle, path/cancel/failure와 clean teardown을
검사하기 위한 검증 도구다. production motion adapter나 실제 Go2 제어 경로가 아니다.

## build·test와 파일 동결

- ROS package: 8개
- `colcon build --symlink-install`: exit 0
- test: 318개, failures 0, errors 0, skipped 5
- 설치 executable: `go2_validation=14`, `go2_nav2=0`
- Ruff: 통과
- Python no-excuse audit: 통과
- checksum manifest: `verification/software_only_freeze.sha256`
- checksum 항목: 267개
- checksum manifest 자기 자신: 의도적으로 제외

checksum manifest는 현재 Git-visible 프로젝트 파일 전체와 Domain 64 localization
`result.json`, Domain 65 Nav2 shadow `summary.json`을 포함한다. `build/`, `install/`,
`log/`와 그 밖의 ignored runtime data는 포함하지 않으며, 지도·bag·외부 fixture의
결합 checksum은 위 두 runtime 결과와 구조화 YAML에 별도로 보존한다.

동결 시 project branch는 `codex/stage12-localization-nav2-shadow`, 기준 HEAD는
`7bb72ae`다. Knowledge branch는 같고 기준 HEAD는 `707b599`다. Stage 13은 두 HEAD를
바꾸지 않은 working-tree 동결이며 commit·push는 별도 요청 전 수행하지 않는다.

## 유지하는 warning·deferred

- 30분 정지 yaw 누적 drift `0.248012 rad`: `warning`
- DimOS occupancy production 후보: 없음, 품질 해결 `open`
- Domain 0 live shadow: Go2 OFF 조건으로 `deferred`
- AGX 최종 고정·이동 전원·케이블 strain·열·센서 시야·footprint: `deferred`
- live sensor/TF fit·지도 정확도·localization 정확도: `unverified`
- 실제 축·StopMove·장애물 회피·목적지 도달: `deferred`
- RealSense IMU: 현재 프로젝트에서 `excluded`

Stage 14를 시작할 때는 이 문서의 software 결과를 재사용하되, 최종 장착 이후 9·10단계
비동작 검증을 새 record로 다시 실행한다. 과거 결과나 이 동결 문서는 새로운 물리
실행 승인으로 사용하지 않는다.
