from glob import glob
from setuptools import find_packages, setup


package_name = 'yzz_ptz_camera'


setup(
    name=package_name,
    version='0.3.7',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
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
    description='ROS 2 V4L2 PTZ camera driver for the YZZ robot.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'yzz_ptz_camera = yzz_ptz_camera_driver.driver:main',
        ],
    },
)
