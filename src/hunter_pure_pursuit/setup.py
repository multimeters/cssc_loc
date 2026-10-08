from glob import glob
from setuptools import setup

package_name = "hunter_pure_pursuit"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CSSC localization maintainers",
    maintainer_email="maintainers@example.com",
    description="Pure Pursuit tracking for Hunter.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "pure_pursuit = hunter_pure_pursuit.pure_pursuit_node:main",
    ]},
)
