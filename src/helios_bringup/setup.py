from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'helios_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
         glob('config/*')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), 
         glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaden Howard',
    maintainer_email='jasenhow@gmail.com',
    description='Launch files for Helios swarm robotics platform',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)