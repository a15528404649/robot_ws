from glob import glob
from setuptools import find_packages, setup

package_name = 'wit_ros2_imu'

setup(
    name=package_name,
    version='0.1.0',
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
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='orin@example.com',
    description='ROS 2 driver for WIT serial IMU modules.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wit_ros2_imu = wit_ros2_imu.wit_ros2_imu:main',
        ],
    },
)
