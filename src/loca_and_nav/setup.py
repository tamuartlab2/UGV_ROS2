from setuptools import setup
import os
from glob import glob

package_name = 'loca_and_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
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
            'localization = loca_and_nav.loca_node:main',
            'speed_pub = loca_and_nav.speed_pub_node:main',
            'obstacle_force = loca_and_nav.obstacle_force_node:main',
            'linear_force = loca_and_nav.linear_force_node:main',
            'spring_damper = loca_and_nav.spring_damper_node:main',
        ],
    },
)
