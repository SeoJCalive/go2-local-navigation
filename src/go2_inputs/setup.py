"""Install shared navigation ingress nodes, contracts and launch composition."""

from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "go2_inputs"
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
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Go2 project maintainers",
    maintainer_email="bi-agx1@invalid.example",
    description="Read-only mapping scan and odometry ingress",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "mapping_cloud_gate = go2_inputs.mapping_cloud_gate_node:main",
        "odometry_adapter = go2_inputs.odometry_adapter_node:main",
    ]},
)
