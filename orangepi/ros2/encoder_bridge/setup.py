from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'encoder_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='tthom',
    maintainer_email='tthom289@users.noreply.github.com',
    description='ROS2 bridge for rotating platform encoder data from Teensy',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'encoder_node = encoder_bridge.encoder_node:main',
            'point_cloud_reconstructor = encoder_bridge.point_cloud_reconstructor:main',
            'timing_analyzer = encoder_bridge.timing_analyzer:main',
            'point_cloud_comparator = encoder_bridge.point_cloud_comparator:main',
        ],
    },
)
