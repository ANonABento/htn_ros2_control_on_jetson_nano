from setuptools import setup
import os
from glob import glob

package_name = 'jetson_motor_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.drivers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson_user',
    maintainer_email='user@jetson.local',
    description='Heat-managed motor control for 12V DC motors with L298N',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = jetson_motor_control.motor_controller_node:main',
        ],
    },
)
