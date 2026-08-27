"""
go2_control 패키지, motion 계약과 읽기 전용 trial recorder를 설치한다.

설치 자체는 node를 실행하거나 Unitree Sport request를 publish하지 않는다.
"""

from typing import Final

from setuptools import find_packages, setup


PACKAGE_NAME: Final = "go2_control"

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
        (f"share/{PACKAGE_NAME}/config", ["config/motion_contract.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 project maintainers",
    maintainer_email="bi-agx1@invalid.example",
    description="Nav2 속도 후보 adapter와 제한 시험용 읽기 전용 recorder",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "motion_adapter = go2_control.motion_adapter_node:main",
            "limited_motion_trial_recorder = go2_control.trial_recorder_node:main",
        ],
    },
)
