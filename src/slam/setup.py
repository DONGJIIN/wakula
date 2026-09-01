"""ament_python 安装元数据：SLAM/Nav2 配置、启动和健康监控。"""

from glob import glob
from setuptools import find_packages, setup

package_name = "slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/behavior_trees", glob("behavior_trees/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DONGJIN",
    maintainer_email="2724596499@qq.com",
    description="SLAM and navigation configuration for the quadruped robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nav2_readiness_monitor = slam.nav2_readiness_monitor:main",
            "navigation_health_monitor = slam.navigation_health_monitor:main",
        ],
    },
)
