from glob import glob
from setuptools import find_packages, setup

package_name = 'yzz_waypoint_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='orin@example.com',
    description='Map-associated multi-waypoint patrol navigation for YZZ robots.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_nav_node = yzz_waypoint_nav.waypoint_nav_node:main',
        ],
    },
)
