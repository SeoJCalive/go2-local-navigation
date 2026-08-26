"""비동작 Nav2 설정과 launch를 설치한다.

이 파일은 package asset만 설치하며 node를 시작하거나 command를 publish하지 않는다.
"""

from glob import glob
from typing import Final

from setuptools import find_packages, setup


PACKAGE_NAME: Final = "go2_nav2"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 project maintainers",
    maintainer_email="bi-agx1@invalid.example",
    description="Go2 Nav2 비동작 구성과 통합 preflight 실행 경계",
    license="Apache-2.0",
    extras_require={"test": ["pytest", "PyYAML"]},
    entry_points={
        "console_scripts": [
            "integrated_preflight = go2_nav2.preflight_runner_node:main",
        ],
    },
)
