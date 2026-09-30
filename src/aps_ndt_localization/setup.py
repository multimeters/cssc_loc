from glob import glob
from setuptools import find_packages, setup

setup(
    name="aps_ndt_localization", version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/aps_ndt_localization"]),
        ("share/aps_ndt_localization", ["package.xml"]),
        ("share/aps_ndt_localization/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="APS localization maintainers", maintainer_email="maintainers@example.com",
    description="Standalone native Autoware NDT localization", license="Apache-2.0",
    entry_points={"console_scripts": [
        "ndt_feedback = aps_ndt_localization.feedback:main",
    ]},
)
