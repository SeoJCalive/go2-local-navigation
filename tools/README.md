# 개발 검증 도구

`check_package_dependencies.py`로 전체 workspace의 직접 의존성과 순환을 검사한다.
`go2_validation`은 fault·shadow·실물 no-goal observer·preflight 실행을 소유한다.
core build에는 포함하지 않고 전체 검증에서 `--base-paths`로 명시한다.
MuJoCo 회귀 하네스와 원시 결과는 `.omo/evidence/w04-boundaries`에 보존한다.
