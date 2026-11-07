import os
from glob import glob
from setuptools import setup

package_name = 'oakd_vslam_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_odom_bridge = oakd_vslam_bringup.px4_odom_bridge:main',
        ],
    },
)
