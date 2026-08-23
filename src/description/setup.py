from glob import glob
import os

from setuptools import find_packages, setup


package_name = "description"

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
        (
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*.urdf") + glob("urdf/*.md"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 project maintainers",
    maintainer_email="bi-agx1@invalid.example",
    description="공식 Go2 URDF와 TF 계약 asset",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
