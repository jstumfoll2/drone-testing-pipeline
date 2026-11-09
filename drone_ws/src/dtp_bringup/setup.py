from glob import glob

from setuptools import setup

package_name = "dtp_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jason Stumfoll",
    maintainer_email="j.stumfoll@gmail.com",
    description="Launch files and backend selection for the drone testing pipeline.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
