"""
bringup 패키지와 읽기 전용 센서 계약을 설치한다.

이 파일은 package metadata, YAML 계약, static TF launch 파일을 설치한다. ROS node를
실행하거나 command를 publish하거나 Go2 service를 호출하지 않는다.
"""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "bringup"

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
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 project maintainers",
    maintainer_email="bi-agx1@invalid.example",
    description="읽기 전용 센서 계약과 canonical URDF 기반 static TF launch 구성",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
