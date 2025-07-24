from setuptools import setup
import os
from glob import glob

package_name = 'thor_hardware2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/firmware', glob('firmware/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Hardware interface package for Thor 6-DOF robotic arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node.py = thor_hardware2.camera_node:main',
            'motor_test.py = thor_hardware2.motor_test:main',
            'system_monitor.py = thor_hardware2.system_monitor:main',
        ],
    },
)

