from setuptools import find_packages, setup

package_name = 'wf_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Wall perception node for TurtleBot 4 wall following',
    entry_points={
        'console_scripts': [
            'wall_perception_node = wf_perception.wall_perception_node:main',
        ],
    },
)

