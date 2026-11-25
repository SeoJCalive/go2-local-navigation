# F02 전체 경로 매핑 주행

전체 13점 mapping에서 정책 저응답과 동쪽 장애물 접촉으로 멈추던 원인,
제어 하한의 측정 근거와 반복 수락 결과를 확인할 때 읽는다.
Nav2의 F01/W06/W07 설정은 변경하지 않는다.

## 정책 응답과 waypoint 제어

전진은 매 조건 새 gravity simulator·Velocity_Down·FIFO를 시작해 측정했다.
명령 8초 중 마지막 4초의 독립 freejoint qpos를 body heading으로 적분하고
시간에 대한 최소제곱 기울기를 계산했다. 각 조건은 80개 표본이다.
ROS odometry나 waypoint 제어식은 속도 oracle로 사용하지 않았다.

| 명령 vx (m/s) | 실제 vx (m/s) |
| --- | --- |
| 0.05 | 0.002183 |
| 0.10 | 0.001600 |
| 0.15 | 0.034998 |
| 0.20 | 0.090215 |
| 0.225 | 0.136504 |
| 0.25 | 0.183017 |
| 0.30 | 0.241909 |
| 0.40 | 0.368514 |

반응이 시작되는 0.15 대신 명령 대비 70% 이상 추종하는 0.25 m/s를 접근 하한으로
선택했다. 0.30 m/s cruise에서 감속하더라도 tolerance 진입 전까지 이 하한을
유지하고, tolerance 진입 후에는 terminal state에서 0 명령을 유지한다.

D01 순수 회전은 명령 -0.3에서 실제 -0.014, -0.5/+0.5에서
-0.362/+0.374 rad/s였다. 따라서 회전 하한은 0.5 rad/s다.
F02의 vx=0.25, wz=-0.5/+0.5 추가 측정은 실제 yaw -0.383750/+0.391418,
실제 vx 0.235851/0.232986이었다.

waypoint runner는 방향 오차 0.1 rad 이상에서 제자리 회전하고, 그 안에서는
직진한다. 작은 이동 중 yaw를 증폭하지 않고 0으로 만든다.
회전 요청은 오차에 비례하되 0.5..설정 angular_speed로 제한한다.
설정 상한 자체가 실측 하한보다 작으면 명시적으로 거부한다.
stall watchdog은 목표 거리 감소 또는 회전 중 방향 오차 감소만 진행으로 인정해,
목표에서 멀어지는 이동으로 timeout이 계속 연장되지 않게 한다.

- 원시 전진·moving-turn 결과: `.omo/evidence/f02-mapping/sweep2/curve.csv`.
- 측정 스크립트: `.omo/evidence/f02-mapping/vx_sweep.py`.
- 순수 회전 근거: `.omo/evidence/d01-yaw/curve.csv`.
- 측정 조건은 실물 Go2 또는 다른 policy의 응답으로 일반화하지 않는다.

## 경로와 접촉 판정

기본 `test/acceptance/route.json`은 원래 북·남·동·서 13점이다.
동쪽 세 점을 `(5,-0.4) → (6.7,-0.4) → (5,-0.4)`로 옮겼다.
이전 중앙 8점은 `route_central8.json`으로 선택한다.

island의 bounds는 x=[5.22,6.18], y=[0.18,1.82] m다.
동쪽 수평선과 island의 중심선 여유는 기존 0.18에서 0.58 m가 됐다.
기존 Nav2 footprint 반지름 0.42 m를 포함하면 island 여유는 0.16 m다.
같은 구간 남쪽 pillar의 중심선 여유는 0.54 m, footprint 여유는 0.12 m다.
전체 경로의 가장 작은 보수적 연속 여유 하한은 0.112501 m다.

`route_clearance.py`는 MJCF의 실제 box/cylinder를 읽는다.
0.01 m 이하 표본 간격의 반을 추가로 빼, 장애물까지 거리의 1-Lipschitz 성질로
표본 사이의 직선 구간까지 포함한다. 이 값은 계획 중심선 기준이다.
실제 추종과 다리 움직임은 simulator의 매 physics step 접촉으로 따로 판정한다.
`contacts.csv`는 world body의 비-plane geom과 robot geom 사이의 모든 접촉을
기록하며, 양의 contact distance도 제외하지 않는다.

## 검증 경계

- bridge 회귀·독립 측정 plant 시험: 65 passed.
- 손계산 edge/corner/회전 box/cylinder와 충돌 음성 대조: 3 passed.
- native acceptance driver의 Release build: exit 0.
- acceptance Python compile과 반복 실행 shell 문법 검사: exit 0.
- 모든 bridge/native build는 `/tmp/f02-*`를 사용한다.
- 현재 W07 core runtime은 `/tmp/w07-close-install`을 읽기 전용으로 사용한다.
- 기본 `install/`, RL·공유 simulator source와 Nav2 YAML은 수정하지 않는다.
- 전체 기존 colcon suite는 다시 실행하지 않았다.

## 긴 epoch의 native clock

첫 전체 실행은 234.652895초에 13점을 완주했고 접촉·정체는 0이었다.
그 뒤 localization은 센서 matched pair가 0인 상태로 시작해 실패했다.
원본 결과는 `.omo/evidence/f02-mapping/run1/`에 보존한다.

native driver의 `mj_step()`이 0.002초를 double에 계속 더하면 약 257초부터
오차가 0.5 ns를 넘어간다. W01 Go2Bridge의 Sport stamp는 `llround(time*1e9)`,
LowState tick은 `round(time/.001)`이므로 두 상태가 1 ns 어긋난다.
현재 LiDAR의 정확한 timestamp 결합은 이 서로 다른 stamp를 합치지 않는다.

수락 simulator는 epoch별 정수 step 수에서 publication time을 계산한다.
물리 적분, 상태 결합 허용오차와 Nav2 설정은 그대로 유지한다.
reset에서 step 수를 0으로 돌려 두 상태가 같은 새 epoch를 쓰게 한다.
`acceptance_clock_test`는 실제 MuJoCo의 빈 model을 두 epoch 각각 600초
진행하고, fixture의 정확한 2 ms 정수 nanosecond oracle와 두 변환을 대조한다.
원래의 누적 방식에서 불일치가 발생해야만 음성 대조도 통과한다.

공유 GUI simulator/W01 patch의 누적 시간 변환은 이 단위에서 수정하지 않았다.
그 실행 경로의 장시간 timestamp 일치는 별도 수정·검증 대상이다.

## 동일 소스 연속 전체 수락 세 번

Domain 41·localhost·multicast off·전용 discovery tag를 사용했다.
bridge는 `cleanup/domain-guard`의 `5daa5e1`을 `/tmp/f02-bridge-install`에 빌드했다.
세 실행 동안 제어 소스·하네스 소스·native binary의 SHA-256이 동일했다.
원본은 `.omo/evidence/f02-mapping/verified{1,2,3}/result.json`이며
`summary.json`, `audit.json`, `source_verified.json`에 집계·소스 근거가 있다.

| 실행 | mapping wall s | 지도 셀 | 비바닥 접촉 | G1 m | G2 m | G3 m | G4 m | G5 m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| verified1 | 230.021675 | 322×241 | 0 | 0.237786 | 0.156123 | 0.144286 | 0.070469 | 0.195800 |
| verified2 | 230.741588 | 320×240 | 0 | 0.155852 | 0.149001 | 0.057432 | 0.088303 | 0.152910 |
| verified3 | 232.634354 | 318×239 | 0 | 0.168737 | 0.126032 | 0.085845 | 0.097765 | 0.135319 |

- 매회 13점 완료·stall 0, 지도 저장·AMCL 전환·5/5 SUCCEEDED였다.
- 오차는 ROS pose와 독립인 native qpos CSV에서 다시 계산했으며 모두 0.3 m 이하다.
- 세 지도 모두 resolution은 0.05 m/cell이다.
- 독립 command subscriber는 mapping nonzero vx=0.25..0.30 m/s,
  nonzero |wz|=0.5..1.0 rad/s, 이동 중 yaw 0을 확인했다.
- 반지름 0.42 m와 실제 qpos 표본의 최소 장애물 여유는 각각
  0.064191/0.072765/0.072273 m였다. 이 값은 20 Hz 표본 기준이다.
- reset 전 stamp 324/330/327초에서 각각 0.040/0.138/0.096초로 역행했고,
  새 cloud·odom 각각 4개 이상과 동일 localization/bridge PID를 확인했다.
- 매회 여섯 lifecycle GetState는 FINALIZED(4), 종료 오류와 TERM/KILL escalation은 0이다.
- 최장 owned stop은 각각 4.748302/4.780252/4.640735초였다.
- 기존 Domain 1 관련 identity 21개의 PID/start tick/argv가 처음부터 끝까지 동일했다.
- 수락 세 번은 매회 owned PID 39개였다. 실패 실행·속도 sweep까지 포함한
  소유 PID 합집합 157개를 최종 확인했고 잔류는 0이었다.
- 모든 owned FIFO는 제거됐다. build와 근거 output은 명시한 `/tmp`·evidence에 보존한다.
- `saved.route.png` 세 장에서 계획 경로와 실제 동선을 직접 확인했다.
- Nav2 YAML SHA-256은
  `01f0680db3d6027462b669314d34645baa6e7965c8d7425ef2877afc866d4d97`로 전후 동일했다.
- world SHA-256은
  `f164137d1b61d2c113df2e6f9fe9f0bb096f5c5085ec34547a2ed6e38ed39f2d`로 전후 동일했다.

지도 외곽에는 fan/streak와 미탐색 영역이 남는다. 이 시험은 지정된 전체 경로의
완주와 다섯 목표를 판정하며, 지도 전체 정확도나 실물 동작을 검증하지 않는다.
reset 뒤 AMCL 재초기화·goal 재개도 판정 범위 밖이다.
원본 RL controller의 joint-position UDP 활성화 로그는 그대로 관측됐으며,
DDS 외 telemetry 송신 경로의 실측·변경은 이 단위에서 하지 않았다.
