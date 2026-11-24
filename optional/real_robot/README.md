# 실물 후보 패키지

기존 실물·비동작 경로를 보존하고 전체 build/test로 확인하는 영역이다.
센서 acceptance는 `go2_sensor`, source probe는 `go2_state_estimation`,
obstacle candidate는 `go2_perception`, Sport 경계는 `go2_control`,
실물 launch 조합과 offline RViz는 `bringup`이 소유한다.
공통 adapter와 scan·static TF는 core package를 재사용한다.
현재 W04 검증은 simulation 전용이며 실물 실행 가능성은 새로 검증하지 않는다.
