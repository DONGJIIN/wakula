"""ament_python 安装元数据：地形安全评估与 Nav2 速度门。"""

from setuptools import find_packages, setup

package_name = "quadruped_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/terrain_navigation.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Quadruped Developer",
    maintainer_email="developer@example.com",
    description="Conservative terrain decision and Nav2 velocity gate.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "terrain_safety_assessor = "
            "quadruped_planning.terrain_safety_assessor:main",
            "navigation_speed_gate = quadruped_planning.cmd_vel_gate:main",
            "traversal_guidance = quadruped_planning.traversal_guidance:main",
        ],
    },
)
