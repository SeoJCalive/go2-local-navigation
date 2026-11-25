# 선택 패키지

실물 입력 관찰·Sport 경계·비동작 launch를 빌드할 때 `real_robot`을 선택한다.
기본 sim 빌드는 `colcon build --base-paths src`이며 전체 빌드는
`--base-paths src optional/real_robot tools/go2_validation`이다.
선택 패키지는 공통 `description`, `go2_inputs`, `go2_nav2`를 소비한다.
sim core가 선택 패키지 또는 validation에 의존하지 않는다.
