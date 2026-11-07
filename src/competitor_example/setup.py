from glob import glob
from setuptools import setup

package_name = 'competitor_example'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='kelvin',
    maintainer_email='kelvindinhnguyen@gmail.com',
    description='Example skybeta competitor node: hover then a 5 m square.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'square_mission = competitor_example.square_mission:main',
        ],
    },
)
