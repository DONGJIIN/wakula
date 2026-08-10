from glob import glob

from setuptools import find_packages, setup

package_name = "quadruped_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Quadruped Developer",
    maintainer_email="developer@example.com",
    description="Lightweight camera and point-cloud terrain perception.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "terrain_analyzer = quadruped_perception.terrain_analyzer:main",
            "vision_obstacle_detector = "
            "quadruped_perception.vision_obstacle_detector:main",
            "perception_fusion = quadruped_perception.perception_fusion:main",
        ],
    },
)
