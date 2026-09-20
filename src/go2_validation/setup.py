from glob import glob
from typing import Final

from setuptools import find_packages, setup

PACKAGE_NAME: Final = "go2_validation"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 project maintainers",
    maintainer_email="bi-agx1@invalid.example",
    description="Go2 software-only 검증 orchestration 도구",
    license="Apache-2.0",
    extras_require={"test": ["pytest", "PyYAML"]},
    entry_points={
        "console_scripts": [
            "integrated_preflight = go2_validation.preflight_runner_node:main",
            "navigation_runtime_preflight = go2_validation.runtime_preflight:main",
            "fault_fixture = go2_validation.fault_fixture_node:main",
            "fault_acceptance = go2_validation.fault_acceptance_runner:main",
            (
                "live_navigation_acceptance = "
                "go2_validation.live_navigation_acceptance_runner:main"
            ),
            "shadow_fixture = go2_validation.shadow_fixture_node:main",
            (
                "nav2_shadow_acceptance = "
                "go2_validation.shadow_acceptance_runner:main"
            ),
        ],
    },
)
