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
