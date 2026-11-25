# G02 벽 근접 매핑

2026-11-25 MuJoCo GUI에서 재현한 지도 오연결, 시도별 종료, 후보 수정의 한계와
제품 복원 상태를 조회할 때 사용한다. 소유자가 분석 종료를 선택하여 G02 수정은
중단했다. 합격 지도·근접 Nav2 목표·수정 후 수락 하니스는 얻지 못했다.
motion gate는 구현하지 않았으며 새 gate 재생과 GUI를 추가 실행하지 않는다.
이 문서는 simulation 결과이며 실장비 성능을 증명하지 않는다.

증거의 기준 디렉터리는 [G02 증거](../../.omo/evidence/g02-wall-hug/README.md)다.
아래 원시 파일은 이 저장소의 비추적 증거 경로로 연결하며 본문에 복제하지 않는다.

## 고정한 시험과 수정 전 결과

- 경로: 실제 MJCF의 접근 가능한 101개 face, 2,191개 raw waypoint, 275.373 m.
- 경로 SHA256: `8b9cbeb85a44c795137447e8350b46aadba1fd21288497dbb1ec12a88207246e`.
- bottom-left island의 네 틈 중앙선과 origin 복귀 loop를 포함한다.
- 환경: AGX 실제 GUI executable, 전용 Xvfb/Mesa, Domain 41, lo 전용 DDS,
  FIFO Xbox 의미의 입력, qpos 피드백 driver, 해상도 0.05 m.
- 열린 벽 계획 여유는 0.28 m envelope와 0.10 m를 더한 중심선 offset 0.38 m다.
  계획 여유는 움직이는 몸체·다리의 실측 여유를 대신하지 않는다.
- 합격 기준: 벽 normal offset p95 ≤ 0.05 m, 최대 offset ≤ 0.15 m,
  doubled/ghost component 각각 0개, normal thickness p95 ≤ 0.15 m,
  같은 stamp의 SLAM-qpos 최대 XY 오차 ≤ 0.10 m.
- 접촉 중단: 15 mm 초과, 5 mm 초과가 50 ms 넘게 연속, 접촉 1초 초과,
  25초간 arc 진행 4 cm 미만, fall.
- Nav2: 현재 inflation을 유지하고 4개 이상 목표 SUCCEEDED, 각 qpos XY 오차 ≤ 0.30 m.
- 수정 전 attempt 6: 전체 경로 완주, 지도 실패, owned 잔류 0.
- 수정 전 벽 offset p95 1.062681 m, doubled 9개, ghost 75개,
  SLAM pose 최대 XY 오차 5.027770 m.
- 실제 벽 구간 몸체/다리 여유 median/p10/min: 0.174460/0.077386/-0.003463 m.
  요구한 약 0.10 m median은 아직 충족하지 않았다.
- 좁은 틈 여유 median/p10/min: 0.441336/0.313884/0.275936 m.
- 해석적 수평 envelope는 회전 모서리에서 보수적이다. 음의 여유를 실제 침투로
  단정하지 않고 아래 실제 contact와 구분한다. 알려진 0.10 m 배치는
  0.100000000 m로 교차 확인했다.
- contact 205 sample/13 episode, 최대 depth 1.005362 mm, 최대 episode 62 ms.
- cross-track p95/max: 0.034259/0.078883 m.
- 최대 driver loop 간격: 전체 0.806775 s, 최초 오류 전 0.037487 s.
  후반 0.81 s 간격의 원인은 규명하지 못했다.

## 시도와 종료 사유

| 실행 | 관찰·종료 사유 | 근거 |
|---|---|---|
| 1 | 주행 전 DDS 준비 timeout. 시험 XML discovery tag와 GUI SDK 기본 tag 불일치 | [01 결과](../../.omo/evidence/g02-wall-hug/attempts/01/result.json) |
| 2 | sim 41.958 s, north_band_mid의 22 µm 스침을 당시 접촉 0 기준으로 중단 | [02 결과](../../.omo/evidence/g02-wall-hug/attempts/02/result.json) |
| 3 | index 1에서 목표 4.7 cm 잔차. 점 수렴의 작은 속도 명령이 RL deadband에 걸려 중단 | [03 결과](../../.omo/evidence/g02-wall-hug/attempts/03/result.json) |
| 4 | 코너 저속 명령에서 25초간 arc 4 cm 미만 진행으로 중단. 이후 gait 하한 보정 | [04 결과](../../.omo/evidence/g02-wall-hug/attempts/04/result.json) |
| 5 | 약 55%, sim 947.458 s, south_band_mid/FL_calf 약 5.925 mm·6 ms tap을 당시 instant 5 mm 기준으로 중단. 부분 지도 오연결과 6~8초 촬영 block 발견 | [05 결과](../../.omo/evidence/g02-wall-hug/attempts/05/result.json), [부분 진단](../../.omo/evidence/g02-wall-hug/diagnosis-5/) |
| 6 | 독립 capture/driver와 갱신 접촉 정책으로 전체 경로 완주. 지도 정확도 실패, 근접 Nav2 단계 미실행 | [06 결과](../../.omo/evidence/g02-wall-hug/attempts/06/result.json), [지도 지표](../../.omo/evidence/g02-wall-hug/attempts/06/map-metrics.json) |
| 7 | 실제 sigma 0.19에서도 새 오연결. sim 420.368 s에 사용자 지시로 종료 | [동결 07 결과](../../.omo/evidence/g02-wall-hug/offline-gate-review/attempt-7/result.json) |

각 GUI 실행의 정리 결과는 owned 잔류 0이었다. 5차 tap은 최종 정책에서는
중단 대상이 아니며, 6차는 최종 접촉 중단 정책을 위반하지 않았다.

## 최초 원인

sim 201.302초에 실제 `140 → 275` 제약을 삽입하며 pose 오차가
0.008837 m에서 2.607120 m로 뛰었다. 그 전 잘못된 일반 제약은 0개였다.
두 node GT 거리는 2.185242 m였고, 제약의 immutable 상대 XY 측정 오차는
3.553811 m였다. 첫 오류 전 driver 최대 간격은 37.487 ms였으므로 이전
6–8초 blocking screenshot 없이 재현된 오연결이다.

두 scan의 주된 동쪽 벽 비율은 valid ray의 40.86%/37.86%, 관측 최소 range는
0.380834/0.342504 m였다. 전체 scan은 단일 벽 퇴화 기준 70%에 해당하지 않고
0.25 m min-range가 첫 원인이라는 증거도 없었다. 실제 몸체/다리 여유는
0.183213/0.086533 m였다.

원인은 서로 다른 장소의 scan을 맞는 loop로 수락한 perceptual aliasing이다.
7차의 실제 새 edge는 `148 → 284`, sim 202.344 s, GT node 거리 2.061753 m,
상대 측정 XY 오차 3.545454 m였다. coarse/fine은 .403722/.473236,
coarse x/y 분산은 .002477/.010929 m², chain은 18로 작은 분산에서도 오연결됐다.
분산값만 낮추는 수정으로 원인을 제거하지 못했다.

7차 최초 경보에 붙은 `140 → 275`는 reader가 이전 실행 파일을 먼저 읽은
하니스 결함이다. 실제 새 edge와 구분하며 원래 경보는
[stale 경보 원문](../../.omo/evidence/g02-wall-hug/offline-gate-review/attempt7-original-alert-stale-edge.json)에 보존했다.

## 15개 실제 삽입 loop

정상 판정은 저장된 LinkInfo 상대 측정을 GT 상대 pose와 대조하여 XY ≤ 0.10 m,
yaw ≤ 5도로 정했다. graph 최적화 뒤 절대 위치만으로 edge 정확성을 판정하지 않는다.
실제 accepted callback은 26개지만 LinkChainToScan의 거리 gate를 통과해 새로
삽입한 loop 제약은 15개다. 표의 추정 거리는 연결 당시 sensor origin 거리이며
barycenter를 사용하는 SDK 거리 gate와 구분한다.

Coarse 분산은 matcher 내부 m²다. chain은 실제 dataset 재생에서 이 event까지
모든 삽입 쌍·candidate pose가 1e-6 이내 일치한 prefix만 복구했다.

| edge | sim s | 판정 | coarse | fine | var x/y m² | chain | GT m | 추정 link m |
|---|---:|---|---:|---:|---|---:|---:|---:|
| 140 → 275 | 201.302 | 오류 | .364375 | .470194 | .00274443/.136073 | 19 | 2.1852 | 1.3700 |
| 771 → 864 | 604.100 | 정상 | .382722 | .481486 | .0125848/.00261286 | 52 | 2.0739 | 2.0458 |
| 1003 → 1042 | 718.100 | 정상 | .446528 | .452625 | .0022395/.0022395 | 34 | .1461 | .1371 |
| 24 → 1052 | 724.300 | 정상 | .397542 | .503708 | .00251546/.00251546 | 17 | .6952 | .6818 |
| 215 → 1178 | 805.898 | 오류 | .456944 | .510764 | 5.4487/.0135316 | 22 | 2.3652 | 2.2677 |
| 422 → 1203 | 827.694 | 정상 | .509708 | .535597 | .0331241/.0087116 | 36 | 2.7376 | 2.7308 |
| 462 → 1281 | 878.498 | 오류 | .426014 | .495528 | .0424636/.0220259 | 76 | 1.6007 | 1.5866 |
| 637 → 1297 | 888.994 | 정상 | .610417 | .630236 | .00163823/.0538489 | 91 | 2.1890 | 2.1841 |
| 1031 → 1322 | 902.796 | 정상 | .439750 | .450028 | .00227402/.00227402 | 11 | 3.0051 | 2.9945 |
| 677 → 1340 | 913.898 | 정상 | .393653 | .483292 | .0119386/.00254031 | 33 | .0331 | .0201 |
| 23 → 1695 | 1157.294 | 정상 | .479944 | .520069 | .00208357/.00946439 | 40 | .2739 | .3044 |
| 4 → 1837 | 1246.792 | 정상 | .435764 | .487500 | .0112528/.0113357 | 17 | .5181 | .4403 |
| 1090 → 1844 | 1250.592 | 오류 | .447222 | .467181 | .0520447/.0160822 | 39 | 1.9022 | 1.8520 |
| 1237 → 1926 | 1304.488 | 오류 | .368569 | .584833 | .643981/.132355 | 29 | 1.0069 | .9056 |
| 1679 → 2088 | 1427.190 | 정상 | .360056 | .464375 | .221775/.00407801 | 15 | 2.6071 | 2.6331 |

원래 chain 로그는 없었다. 순수 dataset 재생은 후반의 optimizer 수치 분기로
13개 prefix까지만 일치했다. 실제 기록된 query sensor pose를 매 scan 복원하자
15개 삽입 쌍 전부와 candidate pose가 최대 1.21e-8 이내 일치해 마지막 두 chain도
복구했다. 이는 기록 기반 재생값이며 GT나 수락 결정은 주입하지 않았다.
원래 모든 fine LinkInfo covariance 3×3와 복구한 coarse covariance 3×3,
추정 prior 거리는 `.omo/evidence/g02-wall-hug/analysis-6/threshold-separation.json`에 있다.

## 6개 오류·10개 정상 제약의 gate 비교

6차 실제 삽입 15개와 7차 실제 삽입 1개를 합쳤다. 아래 수치는 고정 후보를
다시 판정한 결과이며 gate 변경 뒤 전체 graph에서 생길 새 후보까지 검증한 결과가 아니다.
D는 gate가 유지한 마지막 loop 이후 이동 거리이며 첫 loop 전에는 첫 scan부터 센다.
6차는 저장된 odometry, 7차는 원본 odometry가 없어 같은 scan stamp의 qpos 대체 측정이다.

| gate | 오류 제외 /6 | 정상 손실 /10 | 구현 범위 |
|---|---:|---:|---|
| 원래 .35/.45, chain 10, sigma 3.0 | 0 | 0 | 현재 제품 config |
| sigma .19 | 5 | 2 | 철회한 config 실험 |
| coarse >.47 | 6 | 7 | config 후보 |
| fine >=.50 | 4 | 6 | config 후보 |
| chain >=20 | 2 | 4 | config 후보 |
| chain >=40 | 5 | 7 | config 후보 |
| coarse >.42 + sigma .19 | 6 | 5 | config 후보 |
| fine >=.48 + sigma .19 | 6 | 4 | config 후보 |
| 2%*D+0.20 m motion bound | 6 | 3 | 미구현 Karto 후보 |
| 5%*D+0.20 m motion bound | 6 | 3 | 미구현 Karto 후보 |
| 5%*D+0.20 m + sigma .19 | 6 | 4 | 미구현 Karto/config 후보 |
| 10%*D+0.20 m motion bound | 3 | 4 | 미구현 Karto 후보 |
| 근접 scan의 직선 지지율 >=70% 제외 | 0 | 0 | 미구현 Karto 후보 |
| 직선 지지율 >=35%로 완화해 제외 | 6 | 6 | 미구현 Karto 후보 |

- 관측값의 모든 83,521개 응답·분산·chain 경계를 비교했다. 오류 6개를 모두
  제외하는 config-only 조합은 정상도 최소 4개 잃었다.
- 양쪽 scan에 GT 없이 직선을 맞춘 지지율은 최대 58.90%다. 70% 퇴화 기준으로
  제외되는 node가 없어 motion과 조합해도 추가 이득이 없었다.
- 5%*D+0.20 m는 10/50/100 m 주행 뒤 0.70/2.70/5.20 m 보정을 허용한다.
  실제 Go2 drift가 이 경계 안이라는 실측은 없다.
- 이 motion 후보가 잃은 정상은 `24 → 1052`, `422 → 1203`, `1031 → 1322`다.
  앞서 오염된 graph를 복구하는 큰 보정이 포함돼 있다. 첫 오류를 막은 전체 재생에서
  같은 정상 손실이 생길지는 미검증이다.
- 역사적으로 수락된 모든 loop에서 anchor를 reset하면 같은 5% bound의 정상 손실은
  4개다. 위 표는 gate가 유지한 loop만 reset하는 정책이다.
- motion은 `TryCloseLoop`의 pose 변경·edge 삽입 전 검사와 odometry arc 상태를
  추가하는 source patch가 필요하다. scan 퇴화 제외도 source patch가 필요하다.
  두 방식 모두 제품·재생 engine에 구현하지 않았다.

7차 실제 parameter 조회는 sigma .19, 응답 .35/.45, chain 10, 탐색 폭 8을
확인했다. 같은 GUI YAML을 그대로 읽은 2,099 scan 전체 engine 재생도 내부 분산
.0361 m² 적용을 확인했다. `140 → 275`는 제외됐지만 새 loop 13개 중 오류 2개
(`141 → 354`, `477 → 1131`)가 생겼다. 설정 연결 누락이 아니었다.

16개 전체 행·양쪽 scan·9원소 covariance·보정량·이동 거리와 전수 비교는
[오프라인 상세 표](../../.omo/evidence/g02-wall-hug/offline-gate-review/REPORT_KO.md),
[JSON](../../.omo/evidence/g02-wall-hug/offline-gate-review/all-accepted-edges.json),
[전수 경계](../../.omo/evidence/g02-wall-hug/offline-gate-review/exact-config-frontier.json),
[전체 engine 재생](../../.omo/evidence/g02-wall-hug/offline-gate-review/gui-yaml-full-replay-summary.json)을 따른다.
7차 fine LinkInfo covariance는 기록되지 않아 복구값으로 채우지 않았다.

## 중단 결정과 제품 복원

소유자의 분석 종료 선택에 따라 소스와 공유 `install/go2_nav2` YAML을 원래
`loop_match_maximum_variance_coarse: 3.0`으로 되돌렸다. SDK가 제곱하여 내부
한계는 9 m²다. .19 실험은 승인된 수정이나 현재 실장비 권장값이 아니다.

| G02 변경 | 배치·영향 | 종료 상태 |
|---|---|---|
| 공유 SLAM sigma .19 | 제품 source와 공유 install에 적용됐었음 | 양쪽 3.0 복원 |
| simulation 전용 0.5 m 탐색 override | 제품 launch 실험 | 7차 전에 철회, F05 HEAD와 동일 |
| gate 회귀 fixture·pytest CMake 등록 | 제품 test와 build 등록 | 철회, 원래 CMake 재구성 |
| Karto raw scan·candidate·edge 출력 patch | 저장소 안 `karto-source/`, `karto-debug/`, `karto-observed/` 전용 build | 관찰 source patch만 추적 보존, 공유 install 미배치 |
| 초기 loop listener interposer | 저장소 안 `loop-observer-build/`와 전용 preload | 제품 미배치, 최종 native 기록은 위 patch 사용 |
| physics/clearance·tracking camera hook | 저장소 안 전용 `.so`, owned simulator에만 preload | 원래 step 한 번 호출, 관찰·렌더링만 변경, 공유 simulator 미변경 |
| replay builder와 reader executable | 저장소 안 전용 build, GUI YAML/Ceres를 읽는 분석 도구 | 공유 ROS·제품 install 미배치 |
| recorded-prior replay hook | 기록 hint를 복원하는 재생 전용 동작 변경 | GUI·공유 runtime 미배치 |
| pad·capture·pose observer | 저장소 안 하니스 process | 모두 종료, 제품 launch에 연결하지 않음 |
| SIM_SESSION·결과 문서 | 저장소 문서 | 실패와 복원 상태로 정정 |

`/opt/ros/humble`의 `ros-humble-slam-toolbox`는 `dpkg -V` 출력이 없었고
제품 `install/`에는 Karto/slam_toolbox library가 없다. 관찰·재생 binary는
비추적 전용 산출물로 남지만 전역 loader나 shell startup에 등록하지 않았다.
inflation, 제품 simulator binary, F05 LiDAR/TF/Map 수정을 바꾸지 않았다.

## 하니스 결함과 수정 범위

- DDS discovery tag 불일치는 GUI용 lo XML과 `ROS_LOCALHOST_ONLY` 해제로 수정했다.
- 점 수렴·코너 저속 deadband는 같은 reference의 연속 arc 추종과 .30/.24 m/s
  직선/코너 명령으로 바꿨다. clearance는 driver 입력으로 사용하지 않는다.
- 6~8초 blocking screenshot은 독립 capture process와 독립 20 ms pad process로
  분리했다. 거부 촬영은 10초 간격, 수락 촬영은 항상 enqueue한다.
- CSV/JSONL chunk 끝의 미완성 줄을 보류하고 접촉 frame watermark를 정수 µs로
  맞췄다. 실제 episode의 최대 depth·연속 시간을 기준으로 최종 정책을 적용한다.
- 7차 stale edge는 watcher보다 늦게 truncate되는 native stream의 시작 순서 문제다.
  다음 시작 경계에서 이전 stream을 먼저 비우도록 수정했지만 새 GUI 검증은 하지 않았다.
- 읽는 중 깨진 `result.json`은 atomic replace로 고쳤다. 실제 mapper parameter dump
  비교도 추가했으나 두 수정 모두 7차 실행에 소급 적용되지 않았고 새 GUI 검증은 없다.
- accepted callback과 실제 새 edge를 구분하고 immutable 상대 제약으로 GT 판정했다.
  LiDAR 물리 offset을 `/pose`에 더하는 잘못된 분석은 base-frame scan 계약에 맞춰 제거했다.
- 독립 contact/parser/blocked-renderer fixture는 통과했다. 합성 driver의 20 ms 결과를
  실제 전체 주행 주기 보장으로 확대하지 않는다.

## 미해결과 검증 경계

- 벽 근접 perceptual aliasing과 잘못된 loop 삽입은 해결하지 않았다.
- 6차 전체 벽 offset median/p95/max 0.235010/1.062681/1.984945 m,
  normal-profile p95 0.186250 m, doubled 9·ghost 75,
  thickness p95/max 0.223607/0.447214 m, 최대 pose 오차 5.027770 m로 실패했다.
- 실제 벽 여유 median 약 0.174 m는 요청한 약 0.10 m를 충족하지 않았다.
- 후반 최대 driver gap 0.806775 s는 원인 미규명이다.
- G02 합격 지도와 그 지도의 근접 Nav2 4개 목표는 없다. G02 수정 후 수락 하니스도
  실행하지 않았다. F05의 기존 수락 합격은 [F05 기록](F05_GUI_REPAIRS.md)에 별도로 남긴다.
- motion gate·scan 제외·대안 config의 실장비 성능과 새 graph 전체 검증은 미구현·미검증이다.

## 증거

- `.omo/evidence/g02-wall-hug/live/progress.log`, `live/latest.png`.
- `analysis-6/{summary.json,loop-edges.json,threshold-separation.json}`.
- `live/attempt6-{gt-overlay,error-vs-actual-edges}.png`.
- `maps/baseline6-final.{posegraph,data}`, `baseline6-final-factors.jsonl`.
- [동결 7차 입력·parameter](../../.omo/evidence/g02-wall-hug/offline-gate-review/attempt-7/manifest.json).
- [하니스와 관찰 source 설명](../../.omo/evidence/g02-wall-hug/README.md).
