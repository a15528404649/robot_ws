from glob import glob
from setuptools import find_packages, setup

package_name = 'yzz_robotlaunch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
        ),
        (
            'share/' + package_name + '/maps',
            glob('maps/*')
        ),
        (
            'share/' + package_name + '/rviz',
            glob('rviz/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='orin@example.com',
    description='Bringup, localization, mapping and navigation configuration for BUNKER MINI.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'navigation_gate = yzz_robotlaunch.navigation_gate:main',
        ],
    },
)
