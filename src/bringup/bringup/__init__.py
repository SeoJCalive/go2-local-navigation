"""
Go2 로컬 내비게이션 bringup 패키지의 Python package 경계.

이 패키지는 static TF, read-only acceptance, odometry adapter, stationary perception,
offline RViz와 재사용 가능한 preflight observer를 포함한다.
description 패키지의 canonical URDF와 승인된 base → utlidar_lidar static TF를
사용하며, odom → base adapter는 별도 state-estimation node가 담당한다. AGX
runtime 검증 결과는 records/experiments의 실행 기록에 보존한다.
"""
