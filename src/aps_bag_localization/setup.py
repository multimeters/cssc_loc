from glob import glob
from setuptools import find_packages, setup

PACKAGE = 'aps_bag_localization'
setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE + '/config', ['../../config/localization.yaml']),
        ('share/' + PACKAGE + '/config/native', glob('../../config/native/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='APS localization maintainers',
    maintainer_email='maintainers@example.com',
    description='Native Autoware wheel/IMU, gyro, NDT and EKF map fusion replay',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'bag_adapter = aps_bag_localization.adapter:main',
        'fusion_adapter = aps_bag_localization.fusion_adapter:main',
        'trajectory_recorder = aps_bag_localization.recorder:main',
    ]},
)
