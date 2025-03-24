from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'helios_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*.xacro')),
        
        (os.path.join('share', package_name, 'rviz'), 
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaych',
    maintainer_email='jasenhow@gmail.com',
    description='Robot description package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)