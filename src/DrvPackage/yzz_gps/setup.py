from glob import glob
from setuptools import find_packages, setup

package_name = "yzz_gps"

setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages(where="src", exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orin",
    maintainer_email="orin@example.com",
    description="NMEA GPS driver for the YZZ robot.",
    license="MIT",
    entry_points={"console_scripts": ["gps_driver = yzz_gps_driver.driver:main"]},
)
