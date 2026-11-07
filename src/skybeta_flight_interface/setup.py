from setuptools import setup

package_name = 'skybeta_flight_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='kelvin',
    maintainer_email='kelvindinhnguyen@gmail.com',
    description='skybeta PX4 offboard flight interface (ENU setpoints in, '
                'clamped NED TrajectorySetpoints out).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'flight_interface = '
            'skybeta_flight_interface.flight_interface:main',
        ],
    },
)
