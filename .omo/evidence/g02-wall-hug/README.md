# G02 시험 하니스 보존

실패한 벽 근접 매핑의 제어·관찰·분석 방법을 감사할 때 사용한다.
최종 결과와 현재 제품 상태는 [G02 결과 문서](../../../docs/stabilization-2026-11/G02_WALL_HUG.md)가 소유한다.
소유자는 분석 종료를 선택했다. 새 gate 구현·재생·GUI 실행은 승인되지 않았다.

- 추적 대상은 이 디렉터리의 선택된 Python/C++/header와 관찰 patch다.
- 생성 로그·원시 scan·physics CSV·지도·이미지·컴파일 binary·외부 source checkout은 기존 위치에 비추적 증거로 보존한다.
- `gui_run.py`는 같은 장비의 기존 `../g01-owner-gui/owner_gui.py` helper·offset·전용 config를 사용한다. 이 보존본은 독립 배포 도구가 아니다.
- `geometry.py`는 MJCF 경로와 GT 평가, `path_tracker.py`·`pad_driver.py`는 qpos 피드백 FIFO 주행, `contact_policy.py`는 접촉 중단 판정이다.
- `async_capture.py`·`live_capture.py`는 driver와 별도 process에서 촬영한다. `live_pose_watch.py`는 읽기 전용 경보다.
- `physics_observer.cpp`·`clearance_2d.h`는 원래 `mj_step`을 한 번 호출한 뒤 qpos·접촉·해석적 여유를 기록한다. 같은 library의 camera hook은 렌더링만 바꾼다.
- `loop_observer.cpp`는 초기 listener 관찰기다. 실제 최종 기록은 아래 Karto patch를 사용했다.
- `karto-observation.patch`는 slam_toolbox 2.6.10 commit `8293d21fe5c816d0405e0cc1f3eb46c70659c267`의 raw scan·후보·edge 출력만 추가한다. 수락 조건·최적화·scan 입력을 바꾸지 않는다.
- `karto-source/`, `karto-debug/`, `karto-observed/`, `loop-observer-build/`는 이 저장소 안의 비추적 전용 source/build다. `/opt/ros/humble` 또는 제품 `install/`에 이 SDK를 설치하지 않았다.
- `replay_dataset.cpp`는 저장 dataset을 GUI YAML과 같은 `SMapper::configure`로 읽는다. Karto/Ceres include·link flag는 기존 장비 build를 사용한 전용 재생 executable에만 적용됐다. 제품 build·launch의 일부가 아니다.
- `replay_prior_hook.cpp`는 6차 chain 복구 때 실제 기록 query hint를 복원한 **재생 전용 동작 변경**이다. GT·수락 결정을 넣지 않으며 GUI·제품 launch에 연결하지 않았다.
- `read_graph.cpp`·`read_dataset_odom.cpp`는 저장 제약과 odometry를 읽는다. `analyse_edges.py`와 `compare_recorded_gates.py` 등은 보존 증거를 분석한다.
- `test_contacts.py`, `test_loop_events.py`, `test_async_driver.py`, `test_geometry.py`는 독립 fixture 검사다. 앞의 세 파일은 top-level assertion이므로 pytest 수집 0개를 합격 test 수로 세지 않는다.
- stale edge 경보의 시작 경계 수정·atomic result writer·실행 parameter dump는 7차 종료 전후에 작성했으며 새 GUI에서 검증하지 않았다.
- `5%*D+0.20 m` motion gate는 구현하지 않았다. 분석 스크립트의 고정 후보 판정은 새 graph에서의 동작 구현·검증을 대신하지 않는다.
- `index.md`와 `offline-gate-review/REPORT_KO.md`는 당시 작업 기록이다. 그 안의 “검증 중/승인 후” 문구는 현재 실행 승인이 아니다.
