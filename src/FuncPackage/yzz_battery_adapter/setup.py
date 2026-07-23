from glob import glob
from setuptools import find_packages, setup

package_name = "yzz_battery_adapter"

setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orin", maintainer_email="orin@example.com", license="BSD",
    entry_points={"console_scripts": ["battery_adapter = yzz_battery_adapter.adapter:main"]},
)
