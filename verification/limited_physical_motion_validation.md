# 12단계: 제한적 물리 이동 검증 준비 protocol

이 문서는 최종 장착 후 최신 명시 승인으로만 시작할 수 있는 제한적 물리 이동 시험의
준비 protocol이다. 절차·구조화 계획·read-only recorder 준비는 완료됐지만 물리
이동·제어 publish·gate 개방은 수행하지 않았다. 모든 수치 임계값과 candidate limit의
물리 적합성은 `unverified`다.

## 준비 검증 결과

- protocol과 구조화 acceptance plan은 작성 완료했다.
- read-only recorder는 22개 `go2_control` 테스트와 실제 AGX `/odom` 입력 QA를 통과했다.
- 최종 성공 QA는 odometry 5947개, candidate 0개, preview 0개, exit code 0이었다.
- recorder의 publisher는 ROS 기본 `/parameter_events`·`/rosout`뿐이며 control service client와 action client는 없었다.
- 위 결과는 recorder 준비 근거일 뿐 실제 축·StopMove·제동 또는 물리 이동 근거가 아니다.

## 범위와 gate

- 단계 ID: `stage_12_limited_physical_motion_validation`
- 준비 상태: `completed`
- 실제 물리 검증 상태: `deferred`
- 시작 조건: 11단계 최종 장착 acceptance의 실제 기록, 이동 중 외부 tether가 없는 전원, 최신 사용자 승인, 단일 command owner 확인, 시험 공간과 emergency stop 절차 확인
- 금지 조건: 승인 없는 gate 개방, 복수 command owner, `/lowcmd`, stand/walk/posture 전환 추가, RealSense IMU 사용
- 자세 조건: 이 프로젝트는 stand command를 보내지 않는다. 시험에 안정적인 기립 자세가 필요하면 승인된 작업자가 기존 안전 인터페이스로 별도 수행하고 그 상태와 승인 범위를 trial 시작 전에 기록한다.
- command owner: 시험 시작 전에 하나의 process와 하나의 output path만 식별해 기록한다. owner 식별 결과가 없으면 trial을 시작하지 않는다.
- 승인: 실행 직전에 받은 최신 승인의 시간·승인자·범위·철회 조건을 record한다. 과거 record와 문서는 승인으로 간주하지 않는다.

## trial 공통 절차

- 각 trial 전 `output_enabled`와 `physical_validation_approved`의 실제 값을 기록한다. 두 gate가 기본값 `false`이면 recorder-only 상태이며 물리 명령을 보내지 않는다.
- 각 trial은 x, y, yaw 중 하나의 축만 대상으로 한다. 복합 축 command와 자동 navigation은 이 단계에서 사용하지 않는다.
- 출발 기준점, 측정 기준점, 외부 거리/각도 측정 도구, odometry 시작값을 함께 기록한다. 측정 도구의 정확도와 합격 임계값은 `unverified`다.
- `go2_control/limited_motion_trial_recorder`는 candidate, preview, odom만 구독해 future trial JSON artifact를 남길 수 있다. 첫·마지막 odometry와 최신 Move·StopMove preview의 monotonic 수신 시각을 bounded 상태로 보존하며, publisher·service client·control interface를 만들지 않는다.
- 의도 command, preview, 실제 측정 종료 시각, odometry 종료값, StopMove 요청 시각과 정지 관찰 시각을 같은 trial record에 연결한다.

## 축별 순서

1. x축 trial: 한 방향의 제한된 직선 command만 수행하고, 외부 실측 거리와 odometry x/y 변화를 기록한다. 반대 방향은 별도 trial이다.
2. y축 trial: 한 방향의 제한된 횡방향 command만 수행하고, 외부 실측 거리와 odometry x/y 변화를 기록한다. 반대 방향은 별도 trial이다.
3. yaw축 trial: 한 방향의 제한된 회전 command만 수행하고, 외부 실측 yaw와 odometry yaw 변화를 기록한다. 반대 방향은 별도 trial이다.
4. 각 축 trial마다 StopMove를 별도 event로 요청하고, request·관찰·정지 판단의 시간을 기록한다. stopping latency, stopping distance, yaw overshoot의 합격 임계값은 `unverified`다.

## trial 간 gate closure

- 매 trial 종료 뒤 output gate를 닫고, command owner·publisher 수·watchdog 상태·StopMove 관찰·로봇 정지 상태를 다시 확인한다.
- 다음 축 trial은 위 확인의 artifact와 현재 승인이 그 trial 범위를 계속 포함하는지 확인한 뒤에만 시작한다. 승인 범위를 벗어나면 새 승인이 필요하며, 조건이 충족되지 않으면 다음 trial 상태는 `deferred`다.
- watchdog timeout이 발생한 경우 StopMove event와 실제 정지 관찰을 기록하고, 원인 판단 없이 추가 trial을 중단한다.

## 완료 판단과 기록

- 각 trial은 의도 command, single owner, approval, gate 상태, external measurement, odometry comparison, StopMove/watchdog event, 종료 후 gate closure artifact가 모두 있을 때만 개별 `observed` 결과가 된다.
- 외부 측정과 odometry의 거리·yaw 차이는 값만 기록하며, 사전 합의된 threshold가 없으면 pass/fail로 판정하지 않는다.
- 모든 threshold, candidate limits의 물리 적합성, 안전 거리, stopping latency/distance acceptance는 후속 승인과 측정 체계가 정해질 때까지 `unverified`로 유지한다.
