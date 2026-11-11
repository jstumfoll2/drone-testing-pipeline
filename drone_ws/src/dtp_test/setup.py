from glob import glob

from setuptools import find_packages, setup

package_name = "dtp_test"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/scenarios", glob("scenarios/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jason Stumfoll",
    maintainer_email="j.stumfoll@gmail.com",
    description="Scenario definitions, metrics and test runners for the drone testing pipeline.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "smoke_flight = dtp_test.smoke_flight:main",
        ],
    },
)
