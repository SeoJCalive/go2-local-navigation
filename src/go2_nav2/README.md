# go2_nav2

SLAM Toolbox·저장 지도 AMCL·Nav2 설정과 simulation 세션의 guard를 제공한다.
입력 조합은 `go2_inputs`가 소유하며 직접 workspace runtime 의존성도 이 패키지
하나다. 실물 Sport·obstacle·validation 실행을 포함하지 않는다.
현재 simulation 절차는 [SIM_SESSION](SIM_SESSION.md), 전체 구조와 설치는
[루트 README](../../README.md)를 따른다.

| 경로 | 역할 |
| --- | --- |
| `config/nav2_mujoco.yaml` | simulation Nav2 단일 설정·RotationShim·정책 응답 범위 |
| `config/slam_mapping.yaml` | SLAM Toolbox mapping 설정 |
| `config/saved_map_localization.yaml` | Map Server·AMCL 기본 설정; sim 세션은 자동 원점 pose를 끈다 |
| `config/nav2_shadow.yaml` | 합성 shadow와 실제 입력 no-goal observer 설정 |
| `config/navigation_contract.yaml` | 과거 비동작 실험 계약과 근거 |
| `launch/go2_slam_mapping.launch.py` | SLAM 조합; 외부 입력 owner가 있으면 `start_inputs=false` |
| `launch/go2_saved_map_localization.launch.py` | Map Server·AMCL 조합; 외부 입력이면 `start_inputs=false` |
| `launch/go2_nav2_shadow.launch.py` | 합성 fixture용 inert velocity Nav2 |
| `launch/go2_nav2_live_observer.launch.py` | 실제 입력의 no-goal AMCL/Nav2 observer |
| `maps` | 합성 shadow open/blocked fixture와 preview sidecar |
| `behavior_trees` | sim/shadow/live가 쓰는 정상 BT와 검증용 실패 BT |
| `go2_nav2/sim_session.py` | 지도·초기 pose·mode·flock·tf2 prefix 검증 |
| `go2_nav2/sim_session_guard.py` | AMCL 초기화 수락·소유권 감시·종료 FINALIZED 확인 |
| `go2_nav2/tf2_exec.py` | native process preload와 실제 loaded tf2 검사 |
| `go2_nav2/sim_stop.py` | 여섯 FINALIZED 확인 후 지정 launch PID만 종료하는 단일 명령 |

`sim_session_guard`는 이 패키지의 설치 실행 파일이다. 개발 검증 runner는
`tools/go2_validation`에 있으며, controller preview/costmap-only와
`nav2_non_actuating.yaml`은 `optional/real_robot/bringup`에 있다.
static TF는 `description`이 소유한다.

정규 sim 진입점은 `go2_mujoco_lidar/go2_sim_navigation.launch.py`다.
legacy SLAM launch의 `execution_mode=onboard`와 `use_sim_time=true` 기본값은
별개의 조건이므로 live에서는 `use_sim_time=false`와 clock 부재를 명시적으로 확인한다.
Domain 64 replay·65 합성 action·과거 Domain 0 정지 no-goal 관찰은 지도 정확도,
위치 정확도나 실물 목표 도달 근거가 아니다.
과거 패키지 이동과 당시 검증은 [안정화 기록](../../docs/stabilization-2026-11/README.md)에 있다.
