# Simulation 전용 tf2 pin

새 simulation 세션의 SLAM/AMCL/Nav2 native process를 같은 tf2로 시작할 때 사용한다.
pipeline owner인 local_navigation이 source pin과 bootstrap을 소유한다. 형제 bridge의
binary를 복사하거나 bridge checkout에 의존하지 않는다.

- 원본: https://github.com/ros2/geometry2.git
- commit: `404b7224d623d614f18fa9738dbf1716403d857e` (`0.25.24`).
- 범위: `tf2`만 private prefix에 빌드한다. 전역 ROS 설치는 수정하지 않는다.
- 이유: AGX의 0.25.23 costmap 정지 보고에 대한 기존 실행 조건을 재현한다.
  이 pin 자체가 과거 교착 원인의 독립 증명은 아니다.
- 제거 기준: 전역 tf2의 동등 이상 수정 버전에서 같은 세션 검증과 움직이는
  costmap timestamp를 확인한 뒤 이 entry point의 pin 계약을 함께 갱신한다.

ROS Humble 환경을 source한 AGX 셸에서 프로젝트 루트 기준으로 실행한다.

```bash
bash third_party/tf2/setup.sh /tmp/w05
# /tmp/w05-build/tf2-source, /tmp/w05-build/tf2-pin,
# /tmp/w05-install/tf2-pin, /tmp/w05-log/tf2-pin만 사용한다.
```

`tf2_prefix:=/tmp/w05-install/tf2-pin`을 launch에 넘긴다. source commit, package
version, 설치 library SHA-256을 검사하고 native 실행 prefix가 preload를 강제한다.
prefix는 실제 child `/proc/PID/maps`를 검사해 `TF2_LOADED`를 남긴다.
다른 tf2가 동시에 로딩되거나 10초 안에 로딩되지 않으면 실패한다.
기존 bridge의 library와 전역 ROS 설정은 건드리지 않는다.
