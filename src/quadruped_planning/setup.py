from setuptools import find_packages, setup

package_name = "quadruped_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/crossing.yaml"]),
        ("share/" + package_name + "/config", ["config/competition.yaml"]),
        ("share/" + package_name + "/config", ["config/course_waypoints.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Quadruped Developer",
    maintainer_email="developer@example.com",
    description="Obstacle-crossing behavior decision layer.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obstacle_crossing_manager = "
            "quadruped_planning.obstacle_crossing_manager:main",
            "cmd_vel_gate = quadruped_planning.cmd_vel_gate:main",
            "competition_obstacle_manager = "
            "quadruped_planning.competition_obstacle_manager:main",
            "course_waypoint_navigator = "
            "quadruped_planning.course_waypoint_navigator:main",
            "crossing_action_server = "
            "quadruped_planning.crossing_action_server:main",
            "crossing_action_coordinator = "
            "quadruped_planning.crossing_action_coordinator:main",
        ],
    },
)
