"""Install the real-input read-only odometry probe; shared adapter is in go2_inputs."""

from typing import Final

from setuptools import find_packages, setup


PACKAGE_NAME: Final = "go2_state_estimation"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 project maintainers",
    maintainer_email="bi-agx1@invalid.example",
    description="Go2 read-only source odometry probe",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "odometry_probe = go2_state_estimation.odometry_probe_node:main",
        ],
    },
)
