# `go2_nav2`

`go2_nav2`는 비동작 Nav2 runtime asset만 소유한다. local costmap, controller
preview, SLAM mapping launch와 그 설정·map·BT가 대상이다. SLAM/mapping과 synthetic
Nav2 asset은 모두 실험 후보이며, production navigation runtime 또는 실제 motion
승인을 뜻하지 않는다. fault, replay, mapping acceptance, 통합 preflight 같은
software-only 검증 orchestration은 `go2_validation`이 소유한다.

## 폴더 및 파일 구조

```text
go2_nav2/
├── README.md
├── package.xml
├── resource/
│   └── go2_nav2
├── setup.cfg
├── setup.py
├── config/
│   ├── nav2_non_actuating.yaml
│   ├── navigation_contract.yaml
│   └── slam_mapping.yaml
├── launch/
│   ├── go2_controller_preview.launch.py
│   ├── go2_costmap_only.launch.py
│   └── go2_slam_mapping.launch.py
├── maps/
│   ├── shadow_blocked.pgm
│   ├── shadow_blocked.png
│   ├── shadow_blocked.yaml
│   ├── shadow_open.pgm
│   ├── shadow_open.png
│   └── shadow_open.yaml
├── behavior_trees/
│   └── navigate_to_pose_shadow.xml
├── go2_nav2/
│   └── __init__.py
└── test/
    ├── test_navigation_configuration.py
    └── test_shadow_assets.py
```

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `README.md` | runtime-only 경계, 실험 후보 상태와 retained asset을 설명한다. |
| `package.xml` | Nav2 runtime asset을 위한 ROS 2 metadata와 의존성을 선언한다. |
| `resource/go2_nav2` | ament index package marker다. |
| `setup.cfg` | ament Python script 설치 경로를 지정한다. |
| `setup.py` | config, launch, map, BT asset을 package share 경로에 설치한다. validation executable은 등록하지 않는다. |
| `config/nav2_non_actuating.yaml` | local costmap·controller preview의 frame, obstacle source, candidate velocity 제한과 닫힌 gate parameter를 정의한다. |
| `config/navigation_contract.yaml` | 비동작 Nav2 runtime의 source, candidate 상태, 안전 경계와 실험 상태를 구조화한다. |
| `config/slam_mapping.yaml` | SLAM Toolbox mapping의 frame, scan, map 저장과 기본 search parameter를 정의한다. |
| `launch/go2_controller_preview.launch.py` | controller output을 내부 candidate topic으로 remap하고 닫힌 motion adapter preview를 시작한다. |
| `launch/go2_costmap_only.launch.py` | stationary perception·odometry와 local costmap owner만 조합하며 motion adapter와 goal은 시작하지 않는다. |
| `launch/go2_slam_mapping.launch.py` | mapping scan·odometry와 단일 SLAM Toolbox owner를 조합한다. 기본 `execution_mode=onboard`, `continuity_profile=onboard_observe`를 선언하고 하위 launch에 전달한다. |
| `maps/shadow_blocked.pgm` | blocked synthetic navigation 후보의 occupancy raster다. |
| `maps/shadow_blocked.png` | blocked raster를 시각적으로 열어 보기 위한 PNG sidecar다. 원본 PGM을 대체하지 않는다. |
| `maps/shadow_blocked.yaml` | blocked raster의 image, resolution, origin과 threshold manifest다. |
| `maps/shadow_open.pgm` | open/cancel/failure synthetic navigation 후보의 occupancy raster다. |
| `maps/shadow_open.png` | open raster를 시각적으로 열어 보기 위한 PNG sidecar다. 원본 PGM을 대체하지 않는다. |
| `maps/shadow_open.yaml` | open raster의 image, resolution, origin과 threshold manifest다. |
| `behavior_trees/navigate_to_pose_shadow.xml` | synthetic `NavigateToPose` 후보용 recovery·replanning BT 구조다. |
| `go2_nav2/__init__.py` | 이 package가 validation orchestration을 소유하지 않는 Nav2 runtime asset 경계임을 설명한다. |
| `test/test_navigation_configuration.py` | costmap frame·obstacle source·controller 제한과 costmap-only launch의 non-actuating 경계를 검사한다. |
| `test/test_shadow_assets.py` | map raster·BT와 `go2_validation` 소유 scenario config의 연결을 검사한다. |

## 실행 경계

이 package의 launch는 Nav2/SLAM runtime asset만 조합한다. validation executable은
`ros2 run go2_validation <executable>`로 실행하며, fault·replay·mapping
acceptance와 host lifecycle을 `go2_nav2`에서 실행하지 않는다.

`go2_slam_mapping.launch.py`는 현재 replay 중심 검증 이력 때문에
`use_sim_time=true`를 기본으로 선언한다. `execution_mode=onboard` 기본값과 별개이므로
live Go2에서 사용할 때는 `use_sim_time:=false`를 명시하고 clock owner가 없음을
preflight로 확인해야 한다. replay-only TF·scan profile이 onboard mode에서 거부된다는
사실만으로 이 launch 전체가 live-ready라고 판단하지 않는다.

각도 sweep 시각 비교본은 package runtime asset이 아니며,
`.user/img/slam_map/angle_sweep_20260829/`에서 관리한다.
