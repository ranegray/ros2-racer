from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robo_rover'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ian',
    maintainer_email='ian.mcconachie@colorado.edu',
    description='ROS2 package for ArduPilot Rover control and IMU data publishing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'rover_node = robo_rover.rover_node:main',
    ],
},
)


