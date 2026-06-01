from setuptools import find_packages, setup


package_name = "zyarm_dataset_replay"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/dataset_quality_check.launch.py",
            ],
        ),
        (
            f"share/{package_name}/rviz",
            [
                "rviz/quality_check.rviz",
            ],
        ),
    ],
    install_requires=["setuptools", "pandas", "pyarrow"],
    zip_safe=True,
    maintainer="ZyArm Maintainer",
    maintainer_email="maintainer@zyarm.local",
    description="Raw dataset trajectory replay to ros2_control FollowJointTrajectory controllers.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "dataset_replay = zyarm_dataset_replay.replay_node:main",
        ],
    },
)
