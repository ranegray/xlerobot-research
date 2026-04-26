from setuptools import find_packages, setup

package_name = "xle_fake_hardware"

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
    description="Interface-compatible fake hardware for XLeRobot launch, harness, and bag testing.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_sts3215_node = xle_fake_hardware.fake_sts3215_node:main",
        ],
    },
)
