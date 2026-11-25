# 2026년 11월 simulation 안정화 기록

변경 이유, 당시 검증 결과와 남은 경계를 조회할 때 사용한다.
현재 실행은 [루트 README](../../README.md)와
[SIM_SESSION](../../src/go2_nav2/SIM_SESSION.md)을 따른다.
각 기록의 package 경로·caller·미해결 항목은 해당 단위 시점의 사실이다.

| 기록 | 내용 |
| --- | --- |
| [W04_BOUNDARIES](W04_BOUNDARIES.md) | core 4개·optional 5개·tools 1개로 패키지 경계 이관 |
| [W05_SESSION_EVIDENCE](W05_SESSION_EVIDENCE.md) | W05 지도 세션·tf2·당시 bridge 제한 관찰 |
| [W06_CLEANUP](W06_CLEANUP.md) | 미사용 코드·종료/DDS 공통화와 LiDAR 기본값 |
| [W07_STABILIZATION](W07_STABILIZATION.md) | 시험 oracle·종료 원인·실제 반복 수용 결과 |
| [W07_TEST_INVENTORY](W07_TEST_INVENTORY.md) | 시험 파일별 분류와 Release 음성 대조 |
| [F02_MAPPING](F02_MAPPING.md) | 정책 제어 하한·13점 경로·긴 epoch와 세 번의 수용 |
| [F03_RUNTIME](F03_RUNTIME.md) | 영구 dependency·기본 install 경로와 임시 산출물 정리 |
| [F04_GUI_CLOCK](F04_GUI_CLOCK.md) | 공유 GUI 정수 epoch clock·mutex와 장시간 pairing 검증 |
| [F05_GUI_REPAIRS](F05_GUI_REPAIRS.md) | 실제 관절 TF·mesh-free 표시·durable 지도 reset·단일 종료 명령 |
| [G02_WALL_HUG](G02_WALL_HUG.md) | 벽 근접 오연결 재현·gate 비교·수정 중단과 제품 복원 |
| [W08_CLOSEOUT](W08_CLOSEOUT.md) | 문서·아카이브·전체 빌드/test 마감 |

의존성 검증은 [simulator](../../third_party/unitree_mujoco/README.md)와
[tf2](../../third_party/tf2/README.md)에, 재실행 명령은
[acceptance](../../src/go2_mujoco_lidar/test/acceptance/README.md)와
[headless](../../src/go2_mujoco_lidar/test/headless/README.md)에 둔다.
9월 미추적 실험의 보존 위치는 [archive index](../../verification/archive-index.md)다.
