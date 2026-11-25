"""Nav2 설정·launch·BT와 sim AMCL session guard를 ament install에 배포한다."""

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
        (
            f"share/{PACKAGE_NAME}/maps",
            [
                *glob("maps/*.pgm"),
                *glob("maps/*.png"),
                *glob("maps/*.yaml"),
            ],
        ),
        (
            f"share/{PACKAGE_NAME}/behavior_trees",
            glob("behavior_trees/*.xml"),
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Go2 project maintainers",
    maintainer_email="bi-agx1@invalid.example",
    description="Go2 비동작 Nav2 runtime asset",
    license="Apache-2.0",
    extras_require={"test": ["pytest", "PyYAML"]},
    entry_points={"console_scripts": [
        "sim_session_guard = go2_nav2.sim_session_guard:main",
        "sim_stop = go2_nav2.sim_stop:main",
    ]},
)
