from setuptools import find_packages, setup

package_name = "xle_hardware"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rane Gray",
    maintainer_email="ranegray@users.noreply.github.com",
    description="Hardware-facing STS3215 access, calibration loading, joint states, and guarded commands.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_trajectory_guard_node = xle_hardware.joint_trajectory_guard_node:main",
            "scan_bus1 = xle_hardware.scan_bus1:main",
            "assign_bus1_ids = xle_hardware.assign_bus1_ids:main",
            "wiggle_motor = xle_hardware.wiggle_motor:main",
            "calibrate_bus1 = xle_hardware.calibrate_bus1:main",
            "bus1_sts3215_node = xle_hardware.bus1_sts3215_node:main",
            "capture_pose = xle_hardware.capture_pose:main",
            "goto_pose = xle_hardware.goto_pose:main",
            "record_left_arm_run = xle_hardware.record_left_arm_run:main",
        ],
    },
)
