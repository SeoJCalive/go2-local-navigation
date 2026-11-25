# G02 검증 도구의 정식 테스트 편입

2026-11-25 승인된 후속 작업의 구현 범위·검증 근거·남은 한계를 조회할 때 사용한다.
실행 명령과 파일별 입력/출력은 [하니스 runbook](../../src/go2_mujoco_lidar/test/wall_hug/README.md),
원래 오연결 관측은 [G02 기록](G02_WALL_HUG.md)이 소유한다. 기준 source는
`cleanup/sim-stabilization`의 `2087585`, 승격한 원본은 `123f8b7`이다.

## 편입 범위

- 기존 `test/`와 ament pytest/GoogleTest runner에 G02 Python 56개와 C++ 5개를 추가했다.
- 20 ms lookahead pad process의 qpos/FIFO 코드를 G01 AST 추출 의존성 없이 승격했다.
  driver trace에 기록한 모든 wall/sim gap의 최대값과 실제 packet/최종 중립을 대조한다.
  합성 시계에서 실제 main을 실행해 3 ms 작업 뒤 17 ms만 기다리고 절대 20 ms tick을 유지함을 검사한다.
- 접촉 frame의 world/body별 최대 깊이, 정수 µs duration·watermark, stall/fall을
  양쪽 경계에서 검사한다. 작은 tap과 같은 frame의 얕은 접촉점으로 깊은 접촉을
  지우는 경우를 구분한다.
- renderer를 FIFO에 막은 채 실제 독립 pad child를 완주시키고 순서·PNG·잔류 0을 확인한다.
  socket 포화는 즉시 알리고 미전송 건을 집계하지 않는다. close는 ordered EOF로 drain한다.
- 해석적 body/leg 여유는 실제 MuJoCo model/data의 support 함수로 검사한다.
  10 cm 배치·음수 여유·회전 capsule·cylinder·sphere·ellipsoid를 손계산과 대조한다.
- GT 지도 지표는 기존 계산을 재사용하며 정확/이동/이중/ghost/두꺼운 지도의 음성 대조가 있다.
  CLI가 `map-metrics.json`과 overlay를 만든다.
- pose/qpos 비교는 완결된 줄과 GT bracket을 사용한다. 시작 전에 모든 stream을 비우는
  `prepare_run` 순서와 실제 새 edge 선택으로 attempt-7 stale-reader 회귀를 검사한다.
- Karto patch는 정식 하니스 경로에 보존하며 제품 install과 CMake 적용 경로에 연결하지 않는다.
  재생은 실제 edge ID와 fine pose, chain anchor·길이, 9원소 coarse/fine covariance,
  raw scan ID/stamp/720 ray와 저장 LinkInfo 상대 측정을 교차 확인한다.

## 오프라인 재생 결과

- 공식 진입점: `python3 src/go2_mujoco_lidar/test/wall_hug/replay.py`.
- AGX에서 `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`, ROS source/GUI/MuJoCo 없이 실행했다.
- SHA256 검사와 원시 입력 해석·직선 가설·22개 gate 비교를 포함한 runtime은 **8.585141 s**다.
- 6차 실제 새 loop 15개, 상대 측정 기준 오류 5개다.
- 최초 오류는 `140→275`, sim **201.302 s**다. 상대 XY 오차는 3.553811 m다.
- 7차 실제 loop 1개를 합친 오류 6/정상 10으로 REPORT_KO의 22개 gate 행이 모두 일치했다.
- sigma .19는 오류 5 제외/정상 2 손실, 5%*D+0.20 m retained anchor는 오류 6 제외/정상 3 손실이다.
- 이 재생은 고정된 실제 후보의 관측 데이터 재판정이다. 변경 gate 아래 새 graph를
  재최적화하거나 Karto/Ceres matcher를 재실행하지 않는다.

## 고정 후보 gate 비교 표

오류 제외의 분모는 6, 정상 손실의 분모는 10이다. motion은 retained anchor이며
이 gate가 유지한 직전 loop 이후 이동 거리만 D로 사용한다.

| gate | 오류 제외 | 정상 손실 |
| --- | ---: | ---: |
| original | 0 | 0 |
| sigma .19 | 5 | 2 |
| coarse .47 | 6 | 7 |
| fine .50 | 4 | 6 |
| chain 20 | 2 | 4 |
| chain 40 | 5 | 7 |
| coarse .42 + sigma .19 | 6 | 5 |
| chain 20 + sigma .19 | 6 | 5 |
| fine .48 + sigma .19 | 6 | 4 |
| motion 2%+0.20 m | 6 | 3 |
| motion 2%+0.20 m + sigma .19 | 6 | 5 |
| motion 5%+0.20 m | 6 | 3 |
| motion 5%+0.20 m + sigma .19 | 6 | 4 |
| motion 10%+0.20 m | 3 | 4 |
| motion 10%+0.20 m + sigma .19 | 5 | 4 |
| 직선 70% | 0 | 0 |
| 직선 35% | 6 | 6 |
| 직선 40% | 4 | 4 |
| 직선 45% | 2 | 2 |
| 직선 50% | 2 | 1 |
| 직선 55% | 0 | 1 |
| 직선 60% | 0 | 0 |


## 자료 결정

- 약 29 KiB의 `fixtures/measurements.json`만 Git에 넣었다. 실제 상대 측정·GT pair·
  response/covariance·chain 수·odom arc·센서 직선 지지율의 수치 투영이다.
- correct 라벨은 저장하지 않고 매번 GT 상대 pose와 대조해 계산한다. 변경된 상대 측정을
  GT 측정으로 바꾸면 오류 판정이 바뀌는 음성/양성 대조가 있다.
- 대형 자료는 기존 AGX `.omo/evidence/g02-wall-hug`에 보존하고
  `fixtures/dataset.json`의 파일별 SHA256으로 고정했다. 누락은 이유 있는 skip/CLI exit 77,
  해시 불일치는 실패다. 전체 재생 테스트는 작은 투영과 원시 자료의 계산값을 대조한다.
- attempt-6 chain은 기록 prior를 복원한 과거 engine 재생값이다. attempt-7 odometry와
  fine covariance는 기록되지 않았으므로 qpos 대체값·null을 유지한다.

## 검증·잔여물 감사

- 전체 10패키지 suite를 1회 실행했다. colcon 실행 시간은 약 89 s다.
- 첫 호출에서 UNITREE_MUJOCO_ROOT를 누락해 4개 CTest/6개 pytest case가 실패했다.
  문서 명령을 보완하고 실제 실패 이름으로 다시 실행해 모두 통과했다.
  `--rerun-failed`가 승격 전 test index를 재사용해 실제 이름으로 선택을 보정했다.
- 마지막 JUnit에는 **317개: 통과 314, 실패 1, skip 2, error 0**이 있다.
  JUnit을 만들지 않는 `state_contract_test` 1개도 통과했다.
  따라서 중복 제거한 실 case는 **318개: 통과 315, 실패 1, skip 2**다.
- G02는 **61개 전부 통과**다(Python 56 + C++ 5). fixture/raw replay 32개는 9.12 s,
  정책/절대 일정 19개는 0.17 s, 나머지 비동기·지도·pose는 모두 통과했다.
- 남은 기존 실패: `tools/go2_validation/test/test_validation_package_boundary.py:30`.
  이미 HEAD에 있는 `src/go2_nav2/go2_nav2/sim_stop.py`를 허용 목록이 빠뜨린다.
  해당 두 파일은 기준 2087585 대비 변경하지 않았다.
- 기존 skip 2개는 bringup/go2_sensor copyright 검사다.
- 최종 `colcon test-result` 집계는 333/5실패/2skip이다. 이전 full-run의
  `Testing/20261125-1224/Test.xml`에 남은 CTest wrapper 실패 4개와 JUnit의 기존 실패
  1개를 합산한다. 성공한 이름별 재실행은 JUnit에 반영되지만 이 dashboard XML을
  다시 생성하지 않는다. 원래 실패 기록을 삭제하거나 고치지 않았다.
- `wall_hug_clearance_test` build exit 0; 관찰 patch는 전용 source
  `8293d21fe5c816d0405e0cc1f3eb46c70659c267`에 reverse-apply check를 통과했다.
- 모든 소유 driver/capture/pytest/native 자식은 종료했고, 소유 temp 경로를 정리했다.
  감사 결과 활성 하니스 process 0, 소유 /tmp 잔여물 0, 전용 run tmp 잔여물 0이다.
- 공유 `ros-humble-slam-toolbox`의 `dpkg -V` 출력은 전후 모두 비어 있다.
  install SLAM YAML SHA256은 전후
  `adc6456037cc5448d7f4a8237026b57c7a9b7f895e7e8341de68f5d7143b1529`로 같다.
- /tmp 최상위 2478개 이름의 전후 SHA256도
  `dcce01a8179016d3431c76b212eb53fd237e3de8249ef356c430162b4ea3f92d`로 같다.
  기존 다른 임시 파일은 삭제하지 않았다.


## 유지한 경계

- 기존 `.omo/evidence` 스크립트와 원시 증거는 삭제하지 않았다. 대체 목록은 runbook에 있다.
- GUI 전체 orchestration·route 생성·physics/카메라 preload·engine replay builder는
  기존 증거로 남는다. 이번 정식 검사에서 이들을 새 runtime 경로로 승격하지 않았다.
- 새 GUI에서 stale-reader 시작 순서·관찰 patch를 rebuild한 native mapper는 미검증이다.
- 벽 근접 SLAM false-loop, 후반 0.806775 s driver gap, 실제 로봇 성능은 해결·검증하지 않았다.
- 25분 GUI·물리 동작·main merge·bridge 변경·sigma 변경은 수행하지 않았다.
