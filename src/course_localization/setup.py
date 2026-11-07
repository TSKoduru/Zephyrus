from glob import glob
from setuptools import find_packages, setup

setup(
    name='course_localization', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/course_localization']),
        ('share/course_localization', ['package.xml', 'README.md']),
        ('share/course_localization/launch', glob('launch/*.py')),
        ('share/course_localization/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/course_localization/map', glob('map/*.obj')),
        ('share/course_localization/trajectory', glob('trajectory/*.npz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Course team', maintainer_email='course@example.invalid',
    description='Read-only stereo/ToF particle localization and measured-data logging.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['localize = course_localization.node:main',
                                           'follow_course = course_localization.controller_node:main',
                                           'odom_10cm_test = course_localization.odom_10cm_test:main',
                                           'stereo_map_only = course_localization.stereo_map_node:main']},
)
