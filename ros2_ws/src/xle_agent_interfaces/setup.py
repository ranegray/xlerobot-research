from setuptools import find_packages, setup

package_name = "xle_agent_interfaces"

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
    description="Agent-facing interfaces for mediated access to the XLeRobot research stack.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
