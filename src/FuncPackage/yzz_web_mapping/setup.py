from glob import glob
from setuptools import find_packages, setup

package_name = 'yzz_web_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='orin@example.com',
    description='Local browser console for YZZ mapping and Nav2 navigation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_mapping = yzz_web_mapping.web_mapping_node:main',
            'configure_auth = yzz_web_mapping.configure_auth:main',
        ],
    },
)
