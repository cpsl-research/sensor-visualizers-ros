from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'viz_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spencer',
    maintainer_email='spencer.hallyburton@duke.edu',
    description='Detection muxer for image visualization',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "muxer = viz_camera.muxer:main",
            "singlecam = viz_camera.singlecam:main",
            "multicam = viz_camera.multicam:main",
        ],
    },
)