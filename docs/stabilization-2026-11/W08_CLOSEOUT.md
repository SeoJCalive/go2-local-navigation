# W08 문서와 증거 마감

현재 실행 문서, 과거 기록 보존과 마지막 전체 회귀의 범위를 확인할 때 사용한다.
기준 source는 local_navigation `f671b1f`, bridge `5daa5e1`이다.
W08은 문서와 manifest만 변경하며 제품 코드·설정·테스트 동작은 변경하지 않는다.

## 문서와 사실 정합

- 루트 README: 현재 4개 core·5개 실물 후보·1개 tools, 최초 설치와 시작 순서,
  mapping/save/localization·bridge·tf2·격리·한계를 설명한다.
- SIM_SESSION: 현재 nonzero-domain bridge 계약과 운영 절차를 유지한다.
- 변경 이력 5개: `git mv`로 이 디렉터리에 이동했다.
- W05 당시 실행 결과와 W06 호출 조사: `W05_SESSION_EVIDENCE.md`에 보존했다.
- Nav2 README: 이동된 preview/costmap-only와 stale 파일 목록을 현재 소유권으로 교체했다.
- current manifest: source 10개, validation executable 8개, go2_inputs owner,
  validation_enforce profile을 반영했다.
- 과거 8개 package·15개 실행 파일·327 tests는 historical 필드로 보존했다.
- bridge README/runbook: public sim entry·go2_inputs odom·720×8 planar·local_navigation
  tf2 owner·전진/회전 하한 0.25 m/s·0.5 rad/s를 반영했다.
- GUI/RL 예시: 전용 headless discovery tag 대신 실제 lo-only XML을 사용한다.

## 증거 보존

- archive: `/home/bi-agx1/go2_projects/archive/go2_local_navigation/verification_untracked_20260921era/`.
- 원본 988개, 194570447 bytes를 상대 경로 그대로 rename했다. 삭제한 파일은 없다.
- lsof·접근 가능한 FD·argv 사용 참조 0개, 사용 중 skip 0개였다.
- 모든 원본의 이동 전/후 size·SHA-256 대조 988/988 성공.
- 로그인 서비스 일부의 FD 접근 제한은 [archive index](../../verification/archive-index.md)에 명시했다.
- 추적 문서의 이전 원본 경로 직접 참조는 0개였다. Git 제외 저널은 그대로 보존했다.

## 전체 검증

- 새 Release build: `/tmp/w08-build`, `/tmp/w08-install`, `/tmp/w08-log`.
- 전체 범위: `src optional/real_robot tools/go2_validation`, 10 packages, exit 0, 81초.
- 전체 colcon test: `/tmp/w08-test-log`, 10 packages, exit 0, 72초.
- colcon test-result: 260 tests, 0 errors, 0 failures, 기존 skipped 2개.
- colcon 집계는 native CTest wrapper를 포함하며 개별 pytest 수와 동일 단위가 아니다.
- manifest 문서 관련 후속 pytest: 9 passed.
- 직접 의존성 검사: 10 packages, errors 0, dynamic references 4개.
- 문서 검사: 변경 Markdown 17개·상대 링크 62개, package/executable 수와
  추적 문서의 이전 archive 원본 참조 검사 모두 errors 0.
- `/tmp/w08-install`의 core 4개 Python import·설치 launch description 생성과
  share/package.xml lookup을 ROS node 생성 없이 확인했다.
- 이동 전 감사에 담긴 기존 관련 process 20개의 PID 존재·argv가 이동 후 같았다.
- bridge pytest: 65 passed. 최초 source import 경로 누락 수집 오류 뒤
  source PYTHONPATH를 명시해 실행했고 README에도 같은 명령을 기록했다.
- 모든 build/install/log는 `/tmp`에만 생성했다. 기본 install과 기존 실행을 교체하지 않았다.
- docs-only 변경이므로 acceptance를 다시 실행하지 않았다.
  실제 주행의 최신 근거는 F02의 동일 source 세 번 수용 결과다.

## 미확인

공유 GUI simulator의 긴 epoch stamp 불일치, 전체 지도 정확도·다른 policy/world,
reset 뒤 AMCL 재초기화와 목표 재개, 실물 동작과 새 OS/hosted CI는 W08에서
검증하지 않았다. 실행 중인 Domain 1 stack은 이전 install/센서 조건을 유지하므로
새 문서가 현재 실행 binary를 갱신한 결과라는 뜻은 아니다.
