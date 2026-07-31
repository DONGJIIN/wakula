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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Quadruped Developer",
    maintainer_email="developer@example.com",
    description="Obstacle-crossing behavior decision layer.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "obstacle_crossing_manager = "
            "quadruped_planning.obstacle_crossing_manager:main",
        ],
    },
)

