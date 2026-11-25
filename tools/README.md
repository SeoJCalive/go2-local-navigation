# 개발 검증 도구

`check_package_dependencies.py`로 전체 workspace의 직접 의존성과 순환을 검사한다.
`go2_validation`은 fault·shadow·실물 no-goal observer·preflight 실행을 소유한다.
core build에는 포함하지 않고 전체 검증에서 `--base-paths`로 명시한다.
MuJoCo 회귀는 추적된
[`test/acceptance`](../src/go2_mujoco_lidar/test/acceptance/README.md) 진입점을 쓴다.
기존 W04 원시 결과는 `.omo/evidence/w04-boundaries`에 보존한다.
`launch_test_support.py`는 core 시험이 설치 launch의 argument·condition·typed
parameter를 실제로 평가할 때 사용하며 ROS process를 시작하지 않는다.
