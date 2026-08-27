"""ament_python 安装元数据：rosbag 感知标注与准确率评估工具。"""

from setuptools import find_packages, setup

package_name = "quadruped_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Quadruped Developer",
    maintainer_email="developer@example.com",
    description="Rosbag calibration and perception accuracy tools.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_bag_evaluator = "
            "quadruped_tools.perception_bag_evaluator:main",
            "stack_regression = quadruped_tools.stack_regression:main",
        ],
    },
)
