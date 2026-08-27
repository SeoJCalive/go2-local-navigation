# 11단계: 최종 장착 통합 준비 절차

이 문서는 AGX를 Go2에 최종 고정한 뒤 수행할 비작동 통합 확인의 절차와 기록
체크리스트다. 절차와 기록 구조의 준비는 완료됐지만 실제 장착 검증은 보류 상태다. 이
문서 자체와 이전 실행 결과는 장착·전원·ROS 실행 또는 물리 이동 승인이 아니다.

## 범위와 전제

- 단계 ID: `stage_11_final_mount_integration`
- 준비 상태: `completed`
- 실제 장착 검증 상태: `deferred`
- 현재 장착 상태: AGX 최종 장착 `unverified`
- RealSense IMU: 현재 프로젝트 범위에서 `excluded`
- 실제 control publication과 물리 이동: 이 단계의 비목표
- 기준 재실행: 최종 장착 후 9단계와 canonical 10단계 절차를 각각 다시 실행하고, 각 결과 원문을 새 실행 record로 보존한다.

## 시작 전 gate

- 최종 장착 작업을 수행할 담당자와 작업 시점을 기록한다.
- 로봇의 전원·정지 상태와 작업 공간 안전 조건을 현장에서 확인해 기록한다.
- AGX bracket, 체결 부품, connector, 전원선, Ethernet의 실제 구성과 변경 전 사진을 기록한다.
- 이동 시험에 사용할 최종 전원 source·보호 회로·connector 구성을 기록한다. 현재처럼 외부 전원 tether를 사용하는 상태는 정지 검증에만 사용하고 12단계 진입 조건으로 승격하지 않는다.
- 이 문서와 `structured/final_mount_acceptance.yaml`의 미수행 항목이 `deferred` 또는 `unverified`인지 확인한다.

## 장착·배선 체크리스트

- AGX와 bracket의 체결 위치, 체결부 흔들림, 반복 손가락 압력 후 유격을 관찰해 기록한다. 합격 허용치와 체결 torque는 현재 `unverified`다.
- 모든 connector의 잠금 상태, 케이블 출구의 굽힘·마찰·당김 방향, strain relief 존재 여부를 사진과 관찰값으로 기록한다.
- Ethernet과 전원 경로가 관절·덮개·날카로운 모서리·열원·LiDAR 회전/시야 영역을 침범하지 않는지 확인한다. 최소 이격과 bend-radius 기준은 `unverified`다.
- 이동 중 바닥에 끌리거나 외부 고정점에 연결되는 전원 tether 없이 AGX에 안정적으로 전력을 공급할 수 있는지 확인한다. 허용 전압·전류·보호 기준은 실제 전원 설계와 근거가 확인될 때까지 `unverified`다.
- AGX 흡기·배기와 방열 경로가 bracket·케이블·차체에 의해 가려지지 않는지 기록한다. 온도 임계값과 허용 airflow 기준은 `unverified`다.
- 기본 Unitree LiDAR와 기본 front camera의 시야에 AGX, bracket, connector, 케이블이 걸리지 않는지 정면·측면 사진으로 기록한다. 가시성 판정 기준은 `unverified`다.
- AGX, bracket, connector, cable sweep을 포함한 실제 외곽을 바닥면 기준으로 실측한다. 숫자와 footprint polygon은 실측 전까지 기록하지 않고 `deferred`로 둔다.

## 장착 후 비작동 재검증

- 구성 변경 후 실제 장착 사진, 장착 시각, 담당자, 측정 도구와 원본 파일 경로를 새 record에 보존한다.
- 9단계 통합 비작동 preflight를 gate가 닫힌 기본값으로 다시 실행한다. 실행 전에는 `output_enabled=false`, `physical_validation_approved=false`를 확인한다.
- 10단계 stationary smoke·soak는 canonical 실행 record와 동일한 범위로 다시 실행한다. 이 문서는 10단계 결과를 재해석하거나 대체하지 않는다.
- 재실행 중 `/api/sport/request`와 `/lowcmd` publisher가 생성되지 않았는지 확인하고, 실제 motion command를 publish하지 않는다.
- 재실행 결과를 pre-mount 기준과 비교하되, 차이가 있어도 원인을 추정하지 않고 관찰값·artifact path·후속 판단 필요 여부만 기록한다.

## 완료 판단과 기록

- 모든 장착·배선·시야·footprint 측정이 실제 artifact와 함께 기록되기 전에는 `final-mount-verified`로 승격하지 않는다.
- 최종 이동용 전원 구성이 확인되지 않으면 11단계를 완료하거나 12단계로 진입하지 않는다.
- 9단계 및 10단계 재실행이 별도 실행 record와 artifact를 갖기 전에는 통합 재검증을 `passed`로 표시하지 않는다.
- 이 단계가 완료돼도 실제 제어 approval, 축 방향, StopMove 반응, 제동 거리와 navigation motion은 12단계 이후 별도 최신 승인 대상으로 남는다.
