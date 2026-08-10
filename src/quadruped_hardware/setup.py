from setuptools import find_packages, setup

package_name = "quadruped_hardware"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/hardware.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Quadruped Developer",
    maintainer_email="developer@example.com",
    description="Hardware-independent safety and SDK adapter contracts.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "system_safety_supervisor = "
            "quadruped_hardware.system_safety_supervisor:main",
            "mock_sdk_adapter = quadruped_hardware.mock_sdk_adapter:main",
            "mock_hardware_state = quadruped_hardware.mock_hardware_state:main",
        ],
    },
)
