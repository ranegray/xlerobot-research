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
        ],
    },
)
