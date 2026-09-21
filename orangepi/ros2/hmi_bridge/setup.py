from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hmi_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='tthom',
    maintainer_email='tthom289@users.noreply.github.com',
    description='Sensor orchestration and rosbag2 recording bridge for the Pi 5 scanner',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hmi_bridge = hmi_bridge.hmi_bridge:main'
        ],
    },
)
