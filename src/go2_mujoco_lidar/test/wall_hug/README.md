<!-- 목적: G02 단위·기록·native 재생과 drift 진단의 실행 계약을 안내한다.
입력은 고정 raw/GT이며 제품 설정·실물 실행과 구분한다. 지도 기준은 유지한다. -->
# G02 하니스 단위 검사와 오프라인 재생

벽 근접 오연결 조사에서 GUI를 다시 오래 돌리기 전에 제어·관측·판정 경계를
확인할 때 사용한다. 원래 알고리즘은 harness commit `123f8b7`에서 승격했다.
관측 사실과 실패 경계는 [G02 결과](../../../../docs/stabilization-2026-11/G02_WALL_HUG.md)가 소유한다.

## 실행

프로젝트 루트에서 실행한다. Python 검사는 numpy, scipy, Pillow, PyYAML, pytest가
필요하며 AGX 기존 환경을 사용한다. 새 의존성을 설치하지 않았다.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -q \
  src/go2_mujoco_lidar/test/test_wall_hug_*.py

# 기본값은 아래 고정 AGX 증거 경로다. ROS source·GUI·MuJoCo가 필요 없다.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  src/go2_mujoco_lidar/test/wall_hug/replay.py

# 대형 증거 없이 항상 실행 가능한 작은 측정 투영
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  src/go2_mujoco_lidar/test/wall_hug/replay.py --fixture
```

C++ 여유 검사는 기존 ament GoogleTest에 등록되어 있다. 기존 빌드 환경에서
package를 재구성한 뒤 전체 저장소와 같은 runner로 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/bi-agx1/unitree_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash
export UNITREE_MUJOCO_ROOT="$HOME/.local/share/go2-local-navigation-sim/unitree_mujoco"
export MUJOCO_ROOT="$HOME/.local/share/go2-local-navigation-sim/mujoco-3.3.6"
cmake -S src/go2_mujoco_lidar -B build/go2_mujoco_lidar
cmake --build build/go2_mujoco_lidar --target wall_hug_clearance_test -j2
colcon test --base-paths src optional/real_robot tools/go2_validation \
  --executor sequential --return-code-on-test-failure --ctest-args -j1
colcon test-result --verbose
```

## 유지보수 경계

| 파일 | 입력 → 처리 → 출력 | 검사 |
| --- | --- | --- |
| `path_tracker.py` | 고정 waypoint·qpos → arc/lookahead → pad 축 명령 | 직선·코너·역행·종료 |
| `pad_driver.py` | 부모가 넘긴 read-only mem FD·pad FD·route JSON → 독립 20 ms 절대 일정 → trace/status/result | 캡처가 막힌 동안 완주·실제 gap 집계·최종 중립 packet |
| `contact_policy.py` | CSV physics frame → world/body별 최대 깊이·정수 µs 기간 → 중단·지표 | 15 mm, 5 mm/50 ms, 1 s, 4 cm/25 s, fall 양쪽 경계 |
| `async_capture.py` | 순서 보장 SOCK_SEQPACKET → 별도 ImageMagick child → atomic PNG·완료 JSONL | 정확한 FIFO 신호 대기·요청 순서·포화 시 즉시 오류·무잔류 |
| `clearance_2d.h` | MuJoCo model/data → box·다리 support envelope → signed m | 알려진 10 cm·음수·capsule/cylinder/sphere/ellipsoid |
| `geometry.py` | MJCF·저장 지도·world→map transform → GT 비교 → overlay·map-metrics.json | 정확/이동/이중/ghost/두꺼운 지도 대조 |
| `pose_watch.py` | 완결 telemetry 줄 → GT 보간·같은 stamp의 pose 비교 → 최초 0.3 m 경보 | 미완성 줄·GT bracket·실제 새 edge·stale 시작 경계 |
| `replay.py` | edge·raw scan·candidate·qpos·odom → 불변 상대 측정 검증·gate 재판정 → JSON 표 | 최초 오류·15/5·REPORT_KO의 22개 행·해시 변조 음성 대조 |

운전자 JSON spec의 `mem_fd`, `pad_fd`, `d_address`, `qpos_offset`, `time_offset`,
`route_path`, `output_directory`는 부모가 준비한다. 실제 장비 ABI offset을
자동 탐색하지 않는다. 단위 검사는 synthetic memfd와 pad pipe만 사용한다.
실제 전체 주행에서 20 ms를 보장한다는 뜻이 아니며 후반 0.806775 s gap은 미해결이다.
검사에는 고정 sleep을 쓰지 않고 renderer 준비 FIFO, pad packet, bounded process 종료를 기다린다.
포화된 capture socket은 즉시 BlockingIOError로 알리고 미전송 요청을 집계하지 않는다.
종료는 write-half close의 EOF가 모든 대기 packet 뒤에 도착하도록 한다.

`prepare_run(output)`은 이전 증거를 보존한 뒤 **모든 native writer와 reader보다 먼저**
호출한다. native stream을 먼저 비워 7차의 이전 `140→275` 경보가 새 실행으로
들어오는 시작 순서 결함을 막는다. `PoseMonitor`는 읽기 전용이며 live caller가
`Lines`로 완결된 줄을 전달한다. `pose_watch.py DIRECTORY --transform yaw tx ty`는
동일 비교기를 저장 telemetry에 적용한다. 새 GUI 실행에서의 이 순서는 아직 미검증이다.

`geometry.py --world WORLD --map MAP.yaml --transform yaw tx ty --output NEW_DIR`는
`map-metrics.json`과 `gt-overlay.png`를 만든다. map pass는 offset p95≤0.05 m,
max≤0.15 m, doubled/ghost 각각 0, thickness p95≤0.15 m다. pose≤0.10 m·
contact·Nav2 기준은 이 지도 판정에 섞지 않고 원래 G02의 별도 판정으로 남긴다.
빈 지도는 측정 부재 오류이며 합격이 아니다. 수평 envelope의 음수는 보수적 모서리
계산일 수 있어 실제 침투로 단정하지 않는다. world의 직접 worldbody box/cylinder만
GT로 읽으며 다른 MJCF 구성에 대한 일반성은 미확인이다.

## 자료 고정과 재생 의미

- 대형 원본: `/home/bi-agx1/go2_projects/projects/go2_local_navigation/.omo/evidence/g02-wall-hug`.
- `fixtures/dataset.json`은 소비하는 원본 파일별 SHA256을 고정한다. `--dataset`으로
  같은 내용의 다른 디렉터리를 지정할 수 있다. 파일 누락은 pytest에서 명확한 skip,
  CLI는 `SKIP`와 exit 77이다. 파일이 존재하지만 해시가 다르면 실패한다.
- 작은 `fixtures/measurements.json`은 실제 16개 제약의 상대 측정·GT pair·응답·
  9원소 covariance·chain 수·odom arc·센서 직선 지지율을 보존한다. correct 라벨을
  저장하지 않고 매번 상대 XY≤0.10 m/yaw≤5°로 계산한다. 전체 재생은 원시 파일로
  이를 다시 만들고 작은 투영과 수치 대조한다. 대형 scan·CSV·posegraph는 Git에 넣지 않는다.
- `replay.py --export-measurements`는 원시 입력에서 투영을 stdout으로 출력한다.
  fixture 갱신 시 dataset hash·근거·기대 행을 함께 검토해야 한다.
- 실제 `loop && new_edge`만 집계한다. accepted callback 수를 loop 수로 세지 않는다.
  실제 edge ID를 candidate fine pose·chain anchor와 결합하고 저장 LinkInfo 상대 측정과
  9원소 fine covariance를 대조한다. raw scan의 ID·stamp·720 ray도 확인한다.
- 기대값: 6차 첫 오류 `140→275`, sim 201.302 s, 삽입 15개 중 오류 5개.
  7차 실제 `148→284` 1개를 더한 오류 6/정상 10으로 REPORT_KO의 22개 gate 행을 비교한다.
- 6차 candidate는 기록 prior를 복원해 실제 15쌍/pose를 재현한 이전 engine 재생값이다.
  7차 이동 거리는 scan stamp의 qpos 대체값이며 원래 odometry가 아니다. 7차 fine
  covariance는 없으며 null로 남긴다.
- 이 진입점은 **고정 후보의 관측 데이터 재생**이다. Karto/Ceres로 raw scan을 다시
  매칭·최적화하는 engine 재생이나 gate를 바꾼 counterfactual graph 검증은 포함하지 않는다.

## Counterfactual 엔진 재생

2026-11-25 후속 승인 범위의 진입점은 `counterfactual_replay.py`다.
기존 `replay.py`의 고정 후보 재판정과 별도로, 매 gate마다 빈 Karto graph에서
원본 range와 odometry를 순서대로 입력하고 Ceres를 다시 푼다. 거부는 실제 scan의
pose 변경·edge 삽입·최적화보다 앞에서 일어나므로 이후 candidate와 지도도 바뀐다.
GT와 과거 corrected pose·prior·수락 결정은 엔진에 입력하지 않는다.

```bash
source /opt/ros/humble/setup.bash
PREFIX="$PWD/.omo/evidence/loop-gate-replay/engine"
# 최초 한 번만 실행한다. 기존 prefix는 덮어쓰지 않는다.
bash src/go2_mujoco_lidar/test/wall_hug/counterfactual/build.sh "$PREFIX"
ENGINE="$PREFIX/build/engine_replay"
OUT="$PWD/.omo/evidence/loop-gate-replay/runs-isolated"
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  src/go2_mujoco_lidar/test/wall_hug/counterfactual_replay.py \
  --engine "$ENGINE" --output "$OUT" --baseline --jobs 3
# 위 기준 재현을 통과한 뒤에만 sweep을 실행한다.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  src/go2_mujoco_lidar/test/wall_hug/counterfactual_replay.py \
  --engine "$ENGINE" --output "$OUT" --sweep --jobs 3
# X/K/C는 sweep 결과에서 고른 두 gate의 교집합이다.
# 10개 dataset에서 선택한 거리 1 m와 5%D+.3 m 조합:
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  src/go2_mujoco_lidar/test/wall_hug/counterfactual_replay.py \
  --engine "$ENGINE" --output "$OUT" --combined 1 .05 .3 --jobs 3
python3 -m pytest -q src/go2_mujoco_lidar/test/test_wall_hug_counterfactual.py
```

- `counterfactual/build.sh`는 원본 `8293d21...`을 전용 prefix에 복제하고 관찰 patch와
  `karto-gate.patch`만 적용한다. 제품 build/install과 공유 source·SLAM YAML은 바꾸지 않는다.
- Karto와 Ceres는 각각 worker 1개로 고정한다. native executable의 강한 patch-version
  symbol과 전용 SDK 우선 RPATH가 공유 SDK의 우발적 선택을 오류로 드러낸다.
- `fixtures/counterfactual-datasets.json`은 모든 `.data`·qpos·world·parameter SHA256을
  고정한다. 기본 `.data`는 당시 mapper가 처리한 raw range와 원래 odometry를 보존한다.
  당시 movement filter에서 버린 원래 ROS scan 전체를 보존한다는 뜻은 아니다.
- G02 6차는 독립 physics qpos와 비교한다. 9월 자료는 직렬화된 simulation odometry를
  truth로 쓴다. 이 simulation pose는 실물 wheel/leg odometry의 drift 모델이 아니며,
  독립 qpos 파일을 갖춘 실험과 구별한다.
- 정상 loop는 immutable LinkInfo 상대 측정의 GT XY 오차 ≤0.10 m, yaw 오차 ≤5°다.
  `correct_loops_lost`는 gate=none의 정상 **동일 node pair가 부재한 개수**다.
  바뀐 matcher가 다른 정상 anchor를 선택한 경우도 포함하므로 새 정상 삽입 수도 함께 본다.
- 거리 gate는 두 linked node odometry 위치 간 **직선거리** ≤X다. 누적 arc와 다르다.
  correction gate는 현재 fine pose의 XY 보정 ≤kD+c이며 D는 처음 scan 또는 마지막
  실제 새 loop 삽입부터 원본 odometry를 누적한 arc다. 거부·기존 edge callback은 reset하지 않는다.
- sigma는 3.0만 허용한다. map은 동일 Karto occupancy 생성, 0.05 m·min pass 2·
  occupancy .1로 만들고 기존 `geometry.evaluate`를 호출한다. 정렬은 원래 world odom
  좌표의 identity다. 최종 graph에 맞춰 GT를 다시 정렬하지 않는다.
- 결과는 새 graph·PGM/YAML·immutable factor·모든 gate 결정·scan별 online/final pose·
  GT overlay·지도 지표·summary와 aggregate JSON이다. 실패한 실험 산출물은 덮어쓰지 않는다.
- 기준 재현은 첫 오류 `140→275`/201.302 s와 역사적 map p95 ±0.22 m,
  doubled ±4·ghost ±20·online 최대 pose 오차 ±0.55 m를 요구한다.
  후반 일부 node pair 차이는 보존하며 과거 전체 edge를 강제하는 prior hook은 금지한다.
- 정식 pytest는 실제 첫 350 scan에서 none/거리 1.5 m·3%D+.1·10%D+.3 gate의
  삽입·D reset·downstream pose를 검사한다. 거리 gate는 정상 loop 최적화 후의
  edge·factor·pose·PGM byte 일치까지 확인한다. native engine이나
  대형 원본이 없는 CI에서는 이유를 적고 이 native 검사만 skip한다.
- [전체 10개 dataset 비교와 실물 drift 경계](../../../../docs/stabilization-2026-11/G02_COUNTERFACTUAL_REPLAY.md)를
  최종 수치와 선택 근거로 사용한다.
- G02 7차는 raw scan은 있지만 원본 odometry가 없어 실제 odometry 재생에서 제외한다.
  qpos를 엔진 입력으로 대체해 odometry gate의 검증 결과로 제시하지 않는다.

## 현재 graph motion gate·robust loss 후속 비교

후속 비교는 `engine-motion-v2`처럼 **새 전용 prefix**를 사용한다. 원래 10개에
9월 21일 archive의 `sep21-1800`, `sep21-variance022` 두 raw dataset을 추가했다.
원래 10개 집계와 추가 두 반례의 결과는 분리해서 읽는다. 0.22 실행의 원본 scan도
이번에는 같은 sigma 3.0에서 재생하므로 과거 0.22 실행의 정확 재현을 주장하지 않는다.

```bash
source /opt/ros/humble/setup.bash
PREFIX="$PWD/.omo/evidence/loop-gate-replay/engine-motion-v2"
bash src/go2_mujoco_lidar/test/wall_hug/counterfactual/build.sh "$PREFIX"
ENGINE="$PREFIX/build/engine_replay"
OUT="$PWD/.omo/evidence/loop-gate-replay/motion-robust"
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  src/go2_mujoco_lidar/test/wall_hug/counterfactual_replay.py \
  --engine "$ENGINE" --output "$OUT" --baseline --jobs 3
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  src/go2_mujoco_lidar/test/wall_hug/counterfactual_replay.py \
  --engine "$ENGINE" --output "$OUT" --motion-robust --jobs 3
# motion-selection.json의 best_motion k/a와 best_robust loss를 사용한다.
# --motion-combined K A LOSS로 최선 둘의 조합을 같은 12개에 재생한다.
GO2_COUNTERFACTUAL_ENGINE="$ENGINE" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 -m pytest -q src/go2_mujoco_lidar/test/test_wall_hug_counterfactual.py
python3 src/go2_mujoco_lidar/test/wall_hug/counterfactual_profiles.py \
  --runs "$OUT" --world ../go2_nav2_mujoco_bridge/worlds/go2_complex_indoor.xml \
  --output "$OUT/profiles-final"
```

- motion gate의 D는 **현재 corrected base pose**의 연속 odometry-chain 길이와
  실제 삽입된 loop edge 길이로 구성한 무방향 graph 최단거리다. 새 후보 edge는
  아직 포함하지 않는다. 일반적인 local proximity link는 shortcut에 넣지 않는다.
- 매 후보마다 현재 scan pose에서 chain prefix를 다시 계산한다. anchor·query·
  유지 loop 끝점 사이 branch 없는 chain을 압축한 뒤 Dijkstra를 수행한다.
  V개 scan·L개 유지 loop에서 평균 O(V + L log L) 시간과 O(V + L) 공간이며,
  최적화 전 길이를 cache해 다음 후보에 재사용하지 않는다.
- `theta_max=(aD+b)*pi/180`, XY 보정 ≤kD+.2+D*theta_max와
  `abs(wrap(dyaw))≤theta_max`를 동시에 요구한다. 요청의 최종 명시식인
  **D*theta_max**를 사용했으며 sin(theta)로 치환하거나 각도를 degree로 곱하지 않는다.
  k=.03/.05, a=.5/1 deg/m, b=2 deg 전체 네 조합을 실행한다.
- `ceres_loss_function`은 설치된 2.6.10의 parameter·plugin symbol과 같은 버전
  source에서 확인했다. HuberLoss/CauchyLoss는 scale .7로 **모든 covariance로
  whiten한 XY/yaw constraint residual block**에 적용되며 loop 전용 outlier 제거기가
  아니다. replay process의 NodeOptions override만 사용한다.
- `result-decisions.jsonl`은 dyaw·D·XY/yaw bound와 압축 경로 graph를 보존한다.
  Python이 별도 Dijkstra로 전 후보의 D와 수락식을 검사한다. 엔진 로그의
  `path_searches/path_total_us/path_max_us`는 최단경로 계산 비용이며 map·로그 I/O는 제외한다.
- 새 motion 네 조합은 앞선 정상 loop가 있는 `sep21-1800`, loss 두 조합은 G02의
  고정 raw scan으로 두 번씩 350 scan을 처리해
  edge·factor·online/final pose·decision·PGM byte 결정성을 검사한다.
  motion .03/.5와 CauchyLoss 조합도 `sep21-1800`의 두 반복에서 검사한다.
  기존 재최적화 회귀도 유지한다. 늘어난 실제 엔진 실행량에 맞춰 CTest 한도는 900초다.
- baseline의 horizontal/fullclean/wallboth는 기존 관찰 patch로 raw scan을 추가 보존한다.
  `counterfactual_profiles.py`는 원래 per-wall count와 정확 일치를 먼저 확인하고,
  각 doubled run의 normal interval·최종 graph/odometry endpoint 분포·시각별 이동을
  출력한다. metric 정의와 지도를 수정하지 않는다.

## Karto 관찰 patch

`karto-observation.patch`는 slam_toolbox 2.6.10 commit
`8293d21fe5c816d0405e0cc1f3eb46c70659c267`에 대한 repo-only 관찰 patch다.
`G02_SCAN_EVIDENCE`, `G02_EDGE_EVIDENCE`, `G02_CANDIDATE_EVIDENCE`가 가리키는
전용 파일로 raw scan·실제 ID·fine/prior pose·chain·coarse covariance를 기록한다.
공유 install이나 `/opt/ros`에 설치하지 않으며 CMake는 patch를 적용하지 않는다.
이번 검사는 기존 관찰 출력 계약과 저장 LinkInfo를 검증한다. patch를 새로 build한
native mapper의 실행은 이번에 하지 않았다. sigma 3.0과 제품 SLAM 설정은 유지한다.

## 대체된 증거 스크립트

`.omo/evidence/g02-wall-hug/`의 `path_tracker.py`, `pad_driver.py`,
`contact_policy.py`, `async_capture.py`, `clearance_2d.h`, `geometry.py`의 지도 평가,
`live_pose_watch.py`의 비교기, `compare_recorded_gates.py`의 판정,
`enrich_gate_review.py`의 직선 지지율과 `karto-observation.patch`는 위 정식 경로가
유지보수한다. `test_contacts.py`, `test_async_driver.py`, `test_geometry.py`,
`test_clearance.cpp`는 수집 가능한 정식 테스트로 대체했다. 기존 증거는 삭제하지 않는다.

`gui_run.py` 전체 운영 orchestration, route 생성, physics preload/카메라 hook,
이전 listener `test_loop_events.py`, engine replay builder/prior hook은 보존 증거다.
이번 작업은 물리 주행·25분 GUI·false-loop 수정·새 graph 수락을 수행하지 않았다.

## Attempt 9 확장과 drift 진단

`drift_replay.py`는 기존 12개 manifest를 그대로 읽고 별도
`fixtures/drift-datasets.json`의 attempt 9를 13번째 cohort로 추가한다.
결과 JSON은 `original_12`, `attempt_9`, `all_13`을 분리한다.
고정 startup world→map transform을 사용하며 최종 graph에 맞추지 않는다.

```bash
source /opt/ros/humble/setup.bash
H=src/go2_mujoco_lidar/test/wall_hug
E="$PWD/.omo/evidence/loop-gate-drift"
export TMPDIR="$E/tmp" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$TMPDIR"
cmake -S "$H/counterfactual" -B "$E/engine" \
  -DENGINE_SOURCE="$PWD/.omo/evidence/loop-gate-replay/engine-motion-v2/source"
cmake --build "$E/engine" --target engine_replay -j2
timeout 5400 python3 -u "$H/drift_replay.py" \
  --engine "$E/engine/engine_replay" --output "$E/runs" --compare --jobs 3
timeout 2400 python3 -u "$H/drift_replay.py" \
  --engine "$E/engine/engine_replay" --output "$E/runs" --sensitivity --jobs 3
python3 -u "$H/drift_replay.py" --engine "$E/engine/engine_replay" \
  --output "$E/runs" --profiles
GO2_COUNTERFACTUAL_ENGINE="$E/engine/engine_replay" python3 -m pytest -q \
  src/go2_mujoco_lidar/test/test_wall_hug_drift.py
```

위 `ENGINE_SOURCE`는 고정 2.6.10에 기존 관찰·replay patch를 적용한 격리 source다.
없으면 앞 절의 `counterfactual/build.sh`로 새 prefix를 준비한다.
ROS setup은 `set -u`와 호환되지 않으므로 nounset을 켠 셸에서 source하지 않는다.
새 output directory에서 실행하고 완료 산출물을 덮어쓰지 않는다.
민감도 실행은 같은 output의 `none` 기준을 사용하므로 비교 뒤에 실행한다.
집계는 native node ID를 원본 source ID로 복원하므로 movement filter가 node를
건너뛴 경우도 동일 pair를 비교한다. 완료 raw 재생의 집계만 다시 만들 때는
`--summarize comparison` 또는 `--summarize sensitivity`를 사용한다.
중단 후 `--resume`은 dataset·정책·최종 판정 필드가 일치하는 완료 case만 재사용한다.
동일 engine binary와 입력을 유지한 중단 batch에만 사용한다.
미완료 directory는 덮어쓰지 않는다. Native가 거부한 파라미터 대조는 실패 원문을
남긴 별도 이름으로 보존하고 유효한 새 조합은 새 case 이름으로 실행한다.
해상도 .005 m의 smear 허용 범위는 [.0025,.05] m이므로 .03 m를 함께 지정한다.
해상도 효과는 `.01/.03`의 `smear-fine`과 `.005/.03`을 비교한다.

- `pair-k-c`는 fine pose의 XY 보정량 ≤ `k*D+c`이며 D는 linked anchor와 query
  사이 **원본 odometry arc**다. `GO2_REPLAY_ARC=pair`로만 켜지는 진단 정책이다.
- 과거 `(a)`는 마지막 실제 유지 loop 이후 arc다. `retained-0.05-0.2`를 따로
  실행한다. node 간 arc와 같은 정의로 표기하지 않는다.
- 거부된 정상 후보는 실제 native decision과 같은 fine candidate/anchor를
  시간·순서로 연결하고, sensor extrinsic을 제거한 상대 base 측정을 GT와 비교한다.
  `correct_candidates_rejected`는 실제 거부 횟수이며 `correct_loops_lost`는
  none의 정상 동일 pair 부재이므로 서로 바꿔 쓰지 않는다.
- `GO2_REPLAY_ROS_ARGS`는 replay NodeOptions에만 전달한다. scan matching,
  movement filter, correlation, link/match, buffer와 odometry penalty를
  통제하며 공유 YAML·runtime SDK·지도 평가식·수용 threshold는 변경하지 않는다.
- 저장 Dataset에는 이미 movement filter를 통과한 scan만 있다. 낮은
  minimum travel 재생은 원래 버린 ROS scan의 복원이나 더 조밀한 취득 검증이 아니다.
- `wall_hug_drift` CTest는 cohort/grid/extrinsic 계약과 선택적 실제 native
  pair-arc 결정성·matching-off 대조를 검사한다. CI에 native 자산이 없으면 그
  native 검사만 명시 skip하며 simulation 합격으로 승격하지 않는다.
