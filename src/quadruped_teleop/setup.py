"""ament_python 安装元数据：Xbox 手柄到标准 Twist 的安全适配节点。"""

from setuptools import find_packages, setup


package_name = "quadruped_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/xbox.yaml"]),
        ("share/" + package_name + "/launch", ["launch/xbox_teleop.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DONGJIN",
    maintainer_email="2724596499@qq.com",
    description="Fail-safe Xbox Twist adapter with an isolated autonomy process switch.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xbox_teleop = quadruped_teleop.xbox_teleop:main",
        ],
    },
)
