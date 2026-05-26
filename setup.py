from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sen771_agvc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),   glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),   glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'),     glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'),  glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dexter Leong',
    maintainer_email='s223026243@deakin.edu.au',
    description='SEN771 AGVC Path Planning with ROS 2 Jazzy + Gazebo Ionic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = sen771_agvc.planner_node:main',
            'monitor = sen771_agvc.monitor_node:main',
            'debug   = sen771_agvc.debug_node:main',
        ],
    },
)
