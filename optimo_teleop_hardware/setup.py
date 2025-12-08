from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'optimo_teleop_hardware'

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
    install_requires=['setuptools', 'pyspacemouse'],
    zip_safe=True,
    maintainer='optimo',
    maintainer_email='dhk6869@gmail.com',
    description='Hardware interfaces for Optimo robot teleoperation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spacemouse_twist = optimo_teleop_hardware.spacemouse_twist:main'
        ],
    },
)
