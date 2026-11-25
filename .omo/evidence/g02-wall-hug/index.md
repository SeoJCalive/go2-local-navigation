# F05 / G02 증거

소유자가 G02 분석 종료를 선택했다. 공유 sigma는 소스·설치본 모두 3.0으로
복원했으며 새 gate 구현·재생·GUI는 실행하지 않는다. 최종 결과·제품 변경 범위·
미해결 사항은 [G02 결과](../../../docs/stabilization-2026-11/G02_WALL_HUG.md)를 따른다.
아래는 당시 시험·진단의 기록이며 현재 수정 승인이나 실행 기준이 아니다.

AGX의 전용 Xvfb·Mesa·Domain 41·lo와 실제 GUI executable을 사용하는 시험이다.
simulation 결과이며 실물 동작의 근거가 아니다. 원래 simulator config는 직접 바꾸지
않고 동일 inode binary를 전용 실행 경로에 hard link한 뒤 자체 config/FIFO를 사용한다.

- 실시간 화면: `live/latest.png`.
- 약 20초 간격 전체 화면과 milestone: `live/`.
- 단계 기록: `live/progress.log`.
- F05 GUI: `f05-result.json`; G02 시작 전에는 `result.json`.
- G02 driver: `gui_run.py --g02`.
- route 생성·GT 평가: `geometry.py`; 독립 대조는 `test_geometry.py`.
- physics 독립 관찰: `physics_observer.cpp`의 mj_step interposition.
- loop 독립 관찰: `loop_observer.cpp`, 설치판과 같은 Karto 2.6.10
  commit `8293d21fe5c816d0405e0cc1f3eb46c70659c267`의 `KARTO_DEBUG` build.
- 매 step qpos·비발바닥 접촉: `physics-truth.csv`, `physics-contacts.csv`.
- SLAM의 실제 stamped pose: `slam-pose.csv`.
- map와 ground truth overlay: `live/final-map-ground-truth-overlay.png`.

## 실행 전 고정한 합격 기준

- 해상도: 현재 SLAM 설정과 같은 0.05 m.
- 물리 벽 normal profile offset p95: 0.05 m 이하.
- wall offset max: 0.15 m 이하.
- doubled wall: 같은 물리 벽의 같은 면 쪽에 0.10 m 이상 분리된 occupied run이
  벽을 따라 0.20 m 이상 이어진 component가 0개.
- ghost wall: MJCF solid에서 0.15 m 이상 떨어진 occupied component가 0.20 m 이상
  이어진 경우 0개.
- normal profile wall thickness p95: 0.15 m 이하. 넓은 island는 wall thickness에서 제외한다.
- SLAM pose: 첫 정지 pose에서 고정한 world→map 변환을 사용하며 simulator qpos와의
  같은 timestamp XY 오차 최대 0.10 m 이하.
- 비발바닥 접촉: 시간·world geom·robot geom ID·body·depth를 매 step 모두 기록한다.
- 소유자 갱신 기준: depth 15 mm 초과, depth 5 mm 초과가 50 ms 넘게 연속,
  접촉 1초 초과, 경로 진행 stall, fall에서만 중단한다.
- 짧고 얕은 스침은 계속하며 episode 수·contact sample 수·최대 depth·최대 지속 시간을 보고한다.
- Nav2 목표: 4개 이상 SUCCEEDED, 독립 qpos XY 오차 각각 0.30 m 이하.
- 종료: FINALIZED 여섯 개, owned process 잔류 0, 원본 config와 기존 identity 유지.

평가 oracle의 정확한 벽·20 cm shifted wall·추가 wall 대조에서 각각 max offset
0 m·0.165 m·doubled component 1개를 확인했다. 물리 벽 내부 occupied cell을
offset으로 잘못 계산하거나 물리 partition의 양면을 중복 벽으로 세지 않는다.

## 경로와 여유의 해석

- MJCF worldbody의 모든 box와 cylinder에서 접근 가능한 면을 샘플링했다.
- 생성 경로: 101개 면 구간, 약 275.37 m, origin 복귀를 포함한 두 loop.
- 열린 벽 중심선: wall surface에서 0.38 m.
- 계획용 움직이는 body 반경: 0.28 m, 열린 벽 계획 여유 약 0.10 m.
- transit의 회전 중심선 장애물 거리: grid에서 0.36 m 이상.
- 연속 segment의 최소 계획 body 여유: 약 0.0867 m.
- Nav2의 현재 0.42 m 원형 footprint는 변경하지 않는다.
- 정확한 몸체와 다리 여유는 실제 mjData의 geom world 위치와 수평 collision
  support를 world rectangle/circle과 해석적으로 대조한다. 알려진 pose의
  0.10 m 여유는 0.100000000 m로 교차 확인했다. 계획 반경은 실측을 대체하지 않는다.
- bottom-left island의 남·서·동·북 네 틈을 각각 중앙선으로 통과한다.
- 벽과 벽의 접합 내부, 외벽 바깥, 몸체가 들어갈 수 없는 막힌 면은 제외 목록으로 보존한다.

101개 구간의 정확한 이름·방향·제외 면과 모든 waypoint는 `route.json`에 있다.
`live/planned-wall-route.png`에서 초록은 wall traversal, 주황은 transit와 gap이다.

## 결과

G02 실행 결과는 `map-metrics.json`, `mapping-trace.json`, `result.json`에 남긴다.
시험이 실패하면 실제 위치·접촉·stall·부분 지도를 보존하고 완주나 합격으로 기록하지 않는다.
loop closure는 Karto의 실제 ScanID·coarse/fine decision 로그와 `/pose`를 대조한다.

## 시작 실패와 진단

첫 G02는 주행 전에 GUI DDS 준비 시간초과로 종료했고 owned 잔류는 0이었다.
GUI 물리는 57.4초까지 진행됐다. 관찰기 미적용·적용 A/B는 모두 30초간 Sport 수신 0이었다.
하니스 전용 XML은 `go2-w07-acceptance` discovery tag를 사용하지만
일반 GUI SDK participant는 기본 tag를 사용하므로 서로 발견하지 못했다.

- GUI는 `src/go2_mujoco_lidar/config/cyclonedds_loopback.xml`과 Domain 41을 사용한다.
- 이 XML이 lo를 직접 지정하므로 `ROS_LOCALHOST_ONLY`는 해제한다.
- lo XML과 `ROS_LOCALHOST_ONLY=1`을 함께 설정하면 RMW가 lo를 중복 선택해 오류가 났다.
- 올바른 GUI 설정에서 같은 physics 관찰기를 적용한 DDS 준비는 2.775초,
  Sport 수신 11개로 통과했다.
- 원시 진단: `probe-{plain,instrumented}/`, host 실행 기록의 A/B stdout.
- 최초 G02 실행의 원시 물리·로그·result는 재시작 때 `attempts/01/`에 복사했다.

설치판 Karto 2.6.10에는 최신 Humble branch의 `[LoopClosure] ScanID` 출력이 없다.
동일 버전의 기존 debug 출력과 실제 MapperLoopClosureListener callback을 전용 launch에만
적용했다. 원래 `TryCloseLoop`를 정확히 한 번 호출하며 기준값·반환값·pose를 바꾸지 않는다.
coarse/fine 후보 mean, 실제 수락 callback, fine 거부, coarse 기준 미달을 scan의 원래
ID와 timestamp에 연결한다. 시스템 ROS 설치는 변경하지 않았다.

- `test_loop_events.py`: 수락 1개·coarse 거부 1개·fine 거부 1개의 독립 log 대조 통과.
- physics clearance는 wall/obstacle과 robot collision geometry 사이 거리이며 0.75 m에서
  상한을 둔다. floor는 clearance에서 제외하고 접촉은 별도 모든 step에 기록한다.
- 주행 전 자세 준비의 ground 접촉과 매핑 구간 접촉 수를 구분한다.

41.958초 첫 접촉은 north_band_mid의 22 µm 스침이었다. 처음의 접촉 0 기준으로
중단했으며 이후 소유자가 위 중단 기준으로 갱신했다. 원래 driver도 qpos 피드백이었지만
waypoint 0.10~0.12 m 허용과 이동용 0.28 m envelope로 코너 회전을 결정했다.
좁은 waypoint 도달 허용은 RL policy의 저속 deadband에서 멈췄다.
현재는 같은 itinerary를 연속 arc로 투영하고 lookahead·cross-track·heading으로
추종한다. segment에서 최소 전진 속도를 유지하고 코너에서는 감속한다.
stall은 25초 동안 경로 arc 진행이 4 cm 없는 경우다.

## 전체 baseline과 공통 loop 기준 분석

Attempt 6은 독립 촬영과 독립 20 ms pad driver로 275.373 m를 완주했다.
지도는 실패했고 이 결과를 수정 전 기준으로 보존한다.

- 첫 발산: sim 201.302초, pose 오차 0.008837 → 2.607120 m.
- 첫 실제 오류 edge: 140 → 275, GT node 거리 2.185242 m,
  immutable 상대 제약 XY 오차 3.553811 m.
- 첫 발산 전 잘못된 일반 제약: 0개.
- 첫 발산 전 driver 최대 간격: 37.487 ms. 전체 최대는 806.775 ms로 별도 기록한다.
- 실제 accepted callback: 26개. 새로 삽입한 loop 제약: 15개, 그중 오류 5개.
- 벽 offset p95: 1.062681 m. doubled component: 9개. ghost component: 75개.
- SLAM pose 최대 오차: 5.027770 m.
- 실제 벽 구간 여유 median/p10/min: 0.174460 / 0.077386 / -0.003463 m.
  median은 소유자의 약 0.10 m 요구에 아직 미달한다.
- mapping contact: 205 sample, 13 episode, 최대 depth 1.005362 mm,
  최대 episode 62 ms. 갱신 중단 기준 위반은 없었다.
- 증거: `analysis-6/summary.json`, `analysis-6/loop-edges.json`,
  `live/attempt6-gt-overlay.png`, `live/attempt6-error-vs-actual-edges.png`.
- 15개 점수·내부 분산·GT/추정 거리 표: `analysis-6/threshold-separation.json`.
- 원래 로그에는 chain 크기가 없다. 실제 query sensor pose를 복원한 동일
  dataset·Karto·Ceres 재생에서 15개 edge ID 전부와 candidate pose가
  최대 1.21e-8 이내 일치하여 chain 15개를 복구했다. GT나 수락 결정 주입은 없다.

`loop_match_maximum_variance_coarse: 3.0`은 SDK setter에서 제곱되어 내부
9 m² gate가 된다. 당시 공통 값을 0.19로 바꿔 baseline 오류 5개를 제외하고
정상 10개 중 8개를 남겼지만, 7차의 작은 분산 오연결을 막지 못했다.
이 실험은 철회하고 공유 소스·설치본을 3.0으로 복원했다.
simulation의 0.5 m 탐색 제한도 제거했다. 8 m 탐색 폭과 3 m 후보 거리,
.35/.45 응답 gate, 최소 chain 10은 유지한다. G02 합격 지도·Nav2·
post-G02 수락 결과는 없으며 추가 검증은 소유자 선택으로 중단했다.
매 제어의 qpos·target·오차·실제 pad 명령은 `driver-trace.jsonl`로 보존한다.

카메라는 physics를 바꾸지 않는 mjv_updateCamera hook에서 `mjCAMERA_TRACKING`,
`trackbodyid=base_link`로 설정한다. 실제 걸음 중 화면의 중심 유지로 별도 확인한다.
TF2 실행 shim이 LD_PRELOAD를 고정해 초기 loop wrapper는 SLAM child에 전달되지 않았다.
현재는 전용 Karto source의 실제 판정 지점에 출력만 넣고 `karto-observed/`로 빌드했다.

위 점 수렴 시도는 index 1에서 qpos `(0.0384,0.1230)`, target `(0,0.15)`의
4.7 cm 잔차에 멈췄다. 작은 비례 속도 명령으로는 RL gait가 계속 진행하지 않았다.
현재 `path_tracker.py`는 점 도달 판정을 사용하지 않는다.

- 경로 진행: 가까운 다음 segment의 projection arc length를 단조 증가시킨다.
- 추종: 0.22 m lookahead, 코너에서는 0.11 m; cross-track과 heading을 제어한다.
- 전진: 구간에서 0.30 m/s, 코너에서 0.24 m/s; 큰 초기 heading 차이는 먼저 회전한다.
- stall: 25초간 arc progress가 0.04 m보다 늘지 않을 때만 판정한다.
- 벽 여유: `clearance_2d.h`가 실제 body/leg collision geom의 world pose·회전과
  수평 extent를 MJCF box/cylinder에 대조한다. 20 ms 간격의 독립 관측 지표다.
- 거리 backend: `mj_geomDistance`와 CCD 조사는 중단했다. 이전 CCD 기반 수치와
  reference 보정 시도는 현재의 거리·driver 근거로 사용하지 않는다.
- Driver: reference shift와 outward barrier를 제거했다. 원본 `route.json` 계획선을
  추종하며 clearance 값을 입력으로 받지 않는다.
- 형상 근사: 벽 좌표계에 투영한 수평 직사각형 envelope를 사용한다. 열린 벽의
  법선 방향 gap은 해당 extent로 계산하며 회전된 모서리와 원형 장애물 모서리는 보수적이다.
- 빠른 실제 대조: `quick_driver.py`는 기존 acceptance headless physics/RL/FIFO에서
  같은 driver의 첫 세 segment만 실행한다. `headless-first3-*/result.json`에 보존한다.
- 최종 tracking 수치는 waypoint 잔차 대신 cross-track 분포와 실제 geometry clearance다.
- 알려진 10 cm 배치는 `test_clearance.cpp`로 한 번 대조한다.
- 벽 추종 median/p10/min은 실제 physics 시각을 driver command 구간과 맞춰 집계한다.
- 세 벽면 baseline은 6.147/6.185 m, 최대 nominal cross-track 0.0580 m,
  비발바닥 접촉 0, 잔류 0으로 통과했다. 그때의 CCD 기반 거리 median 0.1633 m와
  p10/min 0은 폐기한 계측 결과다. 실제 GUI의 해석적 거리 수치는 별도 결과에 남긴다.
