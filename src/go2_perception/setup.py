"""읽기 전용 perception 패키지와 package metadata를 설치한다.

이 파일은 ROS 2 ament_python 패키지 manifest다. 자체적으로 node를 시작하거나
command를 publish하거나 Go2에 연결하지 않는다. 실행 topic과 frame 계약은
패키지 README와 bringup 설정에 기록한다.
"""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "go2_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [
            os.path.join("resource", package_name),
        ]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 project maintainers",
    maintainer_email="bi-agx1@invalid.example",
    description="읽기 전용 LiDAR obstacle candidate 보고 경계",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obstacle_candidates = go2_perception.obstacle_candidate_node:main",
        ],
    },
)
