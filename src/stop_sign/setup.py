from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stop_sign'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@colorado.edu',
    description='Stop sign detection and event publication for the ROS2 Racer',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stop_sign_node = stop_sign.stop_sign_node:main',
            'stop_sign_visualizer = stop_sign.visualizer:main',
            'stop_sign_snapshot = stop_sign.snapshot:main',
            'cmd_vel_stop_filter = stop_sign.cmd_vel_filter_node:main',
        ],
    },
)
