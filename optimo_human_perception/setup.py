from setuptools import find_packages, setup

package_name = 'optimo_human_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/human_perception.launch.py']),
        ('share/' + package_name + '/models', ['models/pose_landmarker_lite.task']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'human_perception_node = optimo_human_perception.human_perception_node:main',
        ],
    },
)
