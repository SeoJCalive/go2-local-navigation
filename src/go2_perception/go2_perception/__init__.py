"""Go2 로컬 내비게이션의 읽기 전용 perception 패키지.

`obstacle_candidate_node`는 기존 static TF lookup 결과로 `/utlidar/cloud`를
`/perception/obstacle_candidates`에 publish한다. 결과는 obstacle candidate일 뿐
최종 장애물 분류나 free-space 증명이 아니다.
"""
