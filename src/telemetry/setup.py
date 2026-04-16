from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'telemetry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='rane.gray@colorado.edu',
    description='Telemetry aggregator for the ROS2 Racer React dashboard',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'telemetry_node = telemetry.telemetry_node:main',
        ],
    },
)
