from setuptools import setup
import os
from glob import glob

package_name = 'ros2_roboclaw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/ros2_roboclaw.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuan Wei',
    maintainer_email='weiyuan1996@tamu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'roboclaw = ros2_roboclaw.node:main',
        ],
    },
)
