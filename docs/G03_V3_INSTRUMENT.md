# G03 v3 계측기 사전 검증

G03 실행 전에 AGX에서 실제 command publisher와 native 관측기를 연결한
결과다. simulator·RL·실물 로봇은 사용하지 않았다. 기준은 먼저 commit한
`4a2daa79556865bf424da602ddf7391178aa4110`의 v3 절이다.

- 날짜: `2026-11-28`.
- 환경: ROS Humble, `rmw_cyclonedds_cpp`, Domain 41, acceptance의 loopback DDS XML·tag.
- 원자료: `.omo/evidence/g03-saved-map-nav2/instrument-v3-1/`.
- 재현 도구: `src/go2_mujoco_lidar/test/acceptance/command_instrument.py`.
- 판정기: 실제 G03와 같은 `saved_map.command_ownership`.
- 실행: `timeout 300 python3 -u src/go2_mujoco_lidar/test/acceptance/command_instrument.py --observer install/go2_mujoco_lidar/lib/go2_mujoco_lidar/tf_owner_observer --output .omo/evidence/g03-saved-map-nav2/instrument-v3-1`.
- 결과: 네 사례 모두 기대 verdict·개별 메시지 GUID·node 일치, 도구 exit 0.

| 사례 | 실제 명령 수 | 기대 / 실제 판정 | 미식별 | overlap | GUID 귀속 |
| --- | ---: | --- | ---: | ---: | --- |
| (a) controller/behavior 순차 | 2 | PASS / PASS | 0 | 0 | 2/2 정확 |
| (b) 고의 중첩 | 2 | FAIL / FAIL | 0 | 1 | 2/2 정확 |
| (c) 추가 외부 `/g03_external` | 1 | FAIL / FAIL | 0 | 0 | 1/1 정확, 외부 endpoint·sender 검출 |
| (d) 선행 noise 300개 뒤 command publisher 270개 | 270 | PASS / PASS | 0 | 0 | 270/270 정확 |

(a)의 수신 간격은 `.314601193 s`, (b)는 `.012549604 s`다.
(d)는 서로 다른 270개 graph GUID를 생성 때의 graph delta로 확보하고,
메시지마다 고유 숫자 marker를 발행해 예상 GUID와 실제 native 귀속을 대조했다.
metadata는 **608개**이며 256에서 포화하지 않았다. observer exit 0·reap 확인이다.

독립 status 채널은 실제 `/follow_path/_action/status`의 UUID
`01010101010101010101010101010101`의 EXECUTING→SUCCEEDED 구간을 기록했다.
(a)의 두 메시지는 이 구간에 있고 (b)의 두 메시지는 이후에 있다.
(a)의 behavior 발행은 FollowPath 역할과 일치하지 않는다는 별도 진단도 보존했다.
이 사실로 primary 귀속이나 사전 소유권 threshold를 바꾸지 않았다.
behavior spin 구간과 CANCELING·빈 status 배열·unknown primary의 fail-closed
규칙은 unit test에서 별도로 검사했다.

## 원인 재현과 실제 bytes

변경하지 않은 v2 observer에 300개 선행 publication을 주고 controller를 늦게
생성한 `diagnose-v3.py`는 metadata **256개**, 명령 10개 모두 미매핑을 재현했다.
`dds_read`가 처음 256개 instance를 소비하지 않고 계속 반환하므로 이후의
command publication이 metadata batch에 들어오지 않았다. `known` set의
eviction이나 전체 map 크기 제한이 아니다. `dds_take`로 배출하면 위 608개가
기록되고 늦은 270개 publisher도 모두 귀속됐다.

- 원인 재현의 RMW 24 bytes: `958dccd1a6f22ddb00000000000000000000000000000000`.
- 같은 sender의 graph 24 bytes: `01109a2d3940dc8d3b5e2b6100014e030000000000000000`.
- 수정 뒤 (a) controller RMW: `d6affafdd7bd433c00000000000000000000000000000000`.
- 수정 뒤 해당 graph GUID: `0110ca1f96a3771b5bcc7ddc00015d030000000000000000`.
- 수정 뒤 (a) behavior RMW: `cdbae8a338ca4b7700000000000000000000000000000000`.
- 수정 뒤 해당 graph GUID: `0110ca1f96a3771b5bcc7ddc00015e030000000000000000`.

RMW는 8-byte handle과 zero padding이고 graph는 16-byte GUID와 zero padding이다.
서로 길이가 다른 오류가 아니라 이미 존재하던 handle→GUID 연결의 metadata
공급 실패다. 실제 bytes 배열은 `diagnose-v3.json`과 `known_graph.json`에 있다.

## 원자료 SHA256와 관련 검사

| 원자료 | SHA256 |
| --- | --- |
| `diagnose-v3.json` | `23aa14145534bae58dd302fb6efe1dccdd0e4d2753e634e9728cb2dc0cc4266f` |
| `diagnose-v3-native.log` | `c1198d3ba2a10af54535824bd98a875636655e43a98bf1e7a38dc872a8d6cb47` |
| `instrument-v3-1/summary.json` | `ea3e1daf0c87c9ef7fa5ba34c1c8a6350e8f214528731a56eab239b5fa8d5c8a` |
| `instrument-v3-1/native.log` | `365c16ed77f3c8ef0fb377129eb57a58a0845350d95714f6e483d00765ddd34a` |
| `instrument-v3-1/known_graph.json` | `fb8b7e12b647df79092c1b124f8fba02dd82479586be3c3e06b74d33e3eb14a8` |

- native acceptance와 전체 colcon 10개 package build: exit 0.
- 관련 CTest: posture, clearance, process, saved-map, shutdown, session 6/6 통과.
- saved-map unit: 11개 통과, 300개 GUID·정확한 `.25 s` 경계·unknown·외부·상태 구간 포함.
- `colcon test-result`: 누적 364 tests, errors 0, failures 0, 기존 skip 2.
- 첫 CTest 명령의 install overlay 누락과 다음 명령의 `UNITREE_MUJOCO_ROOT` 누락은
  환경을 채운 뒤 실패한 항목만 재검사했다. 초기 로그도 보존했다.
- native physics binary는 v2와 byte 동일하다. SHA256:
  `8ed438254da5305ae6de50ec8fa3c8e1043b6377d6d36612dd816acfaacacabf`.

위 결과는 계측 계약의 증명이며 실제 G03 목표 실행 결과가 아니다.
G03는 구현 commit의 CI green을 확인한 뒤 별도 fresh session에서 한 번 실행한다.
