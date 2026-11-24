# 9월 미추적 실험 증거 보관

2026-09 시뮬레이션 실험의 원시 로그·지도·관측 파일을 찾을 때 사용한다.
현재 운영 절차와 채택한 추적 checkpoint는 프로젝트에 유지한다.

- 보관 target: `go2_agx`.
- 보관일: `2026-11-24`.
- archive: `/home/bi-agx1/go2_projects/archive/go2_local_navigation/verification_untracked_20260921era/`.
- 원본: `go2_local_navigation`의 Git 미추적 `verification/` 파일.
- 이동: `988`개, `194570447` bytes.
- 사용 중 참조로 건너뛴 파일: `0`개.
- 방식: 같은 파일시스템의 rename; archive 아래 `verification/...` 상대 경로 보존.
- 무결성: 원본 이동 전 path·size·SHA-256 기록과 이동 후 전체 `988/988` 대조 성공.
- `MANIFEST.tsv`: `path`, `size`, `sha256` 열; 이동한 원본 988개만 수록한다.
- `USAGE_CHECK.json`: 이동 직전 lsof·접근 가능한 같은 사용자 `/proc/*/fd`와 argv 검사.
- `RESULT.json`: 이동·건너뜀·검증 수와 byte 합계.

예를 들어 과거 `verification/slam_loop_debug_live_20260921_1800/...`는
`<archive>/verification/slam_loop_debug_live_20260921_1800/...`에서 찾는다.
추적 문서에서 이동된 원본의 이전 경로를 직접 참조하는 항목은 없었다.
원본 내부 로그·JSON과 Git 제외 로컬 저널에 기록된 당시 절대 경로는 보존했다.
이전 경로를 읽을 때는 위 상대 경로 대응을 적용한다.

lsof 출력과 접근 가능한 FD/argv에서 보관 파일의 사용 참조는 없었다.
기존 Domain 1 simulator·RL·LiDAR·Nav2·RViz의 argv/FD도 검사했고 signal을 보내지
않았다. 다른 저장소인 bridge의 현재 map 경로는 이번 이동 대상이 아니다.
같은 사용자로 표시되는 non-dumpable `sd-pam`·`sshd`·`sftp-server`의 cwd/FD는
권한상 열람하지 못했다. 해당 argv와 접근 오류는 `USAGE_CHECK.json.denied`에
그대로 보존한다. root/다른 사용자의 비공개 FD까지 검증한 결과는 아니다.

보관 자료를 다시 분석할 때는 이 manifest로 provenance를 확인한다.
현재 package 구성과 실행 가능성은 [루트 README](../README.md)와
[안정화 기록](../docs/stabilization-2026-11/README.md)을 따른다.
