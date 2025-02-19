from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'virtual_gps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Miller',
    maintainer_email='jjm267@pitt.edu',
    description='Simulated virtual GPS with drones - CS 1980 Capstone project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_control = virtual_gps.drone_control:main',
            'virtual_gps = virtual_gps.virtual_gps:main',
            'sensor = virtual_gps.sensor:main',
            'tf_pose_broadcaster = virtual_gps.tf_pose_broadcaster:main',
            'mouse_control = virtual_gps.mouse_control:main',
        ],
    },
)
