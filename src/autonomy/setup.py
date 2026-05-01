from setuptools import find_packages, setup

package_name = "autonomy"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="avery.wagner@colorado.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "wall_nav_node = autonomy.wall_nav_node:main",
            "wall_follower_node = autonomy.wall_follower_node:main",
            "odometry_node = autonomy.odometry_node:main",
            "imu_adapter_node = autonomy.imu_adapter_node:main",
            "path_recorder_node = autonomy.path_recorder_node:main",
            "pure_pursuit_node = autonomy.pure_pursuit_node:main",
            "slam_coordinator_node = autonomy.slam_coordinator_node:main",
            "path_planner_node = autonomy.path_planner_node:main",
        ],
    },
)
