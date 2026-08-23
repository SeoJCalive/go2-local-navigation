"""
읽기 전용 Go2 센서 계약 패키지의 설치 metadata를 정의한다.

이 패키지는 message 계약과 읽기 전용 LiDAR acceptance node를 소유한다. motion
command를 publish하거나 Unitree control service를 호출하지 않는다.
"""

from setuptools import find_packages, setup

PACKAGE_NAME = "go2_sensor"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME],
        ),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 project maintainers",
    maintainer_email="bi-agx1@invalid.example",
    description="읽기 전용 센서 계약·수용 패키지",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "lidar_acceptance = go2_sensor.lidar_acceptance_node:main",
        ],
    },
)
