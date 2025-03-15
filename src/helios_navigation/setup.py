from setuptools import setup
import os
from glob import glob

package_name = 'helios_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.odometry'],  # includes helios_navigation and helios_navigation.odometry
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaych',
    maintainer_email='jasenhow@gmail.com',
    description='Navigation system for Helios robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odom = helios_navigation.odometry.wheel_odometry:main',
        ],
    },
)