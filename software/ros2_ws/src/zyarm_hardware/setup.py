from setuptools import find_packages, setup


package_name = "zyarm_hardware"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/arm_system.launch.py"]),
        (
            f"share/{package_name}/config",
            [
                "config/single_slave_real.yaml",
                "config/teleop_pair_real.yaml",
            ],
        ),
    ],
    install_requires=["setuptools", "pyserial", "PyYAML"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@example.com",
    description="ROS 2 real hardware bridge for ZyArm over serial.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_system = zyarm_hardware.node:main",
        ],
    },
)
