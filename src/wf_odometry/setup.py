from setuptools import find_packages, setup

package_name = 'wf_odometry'

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
    description='Odometry logger node for TurtleBot 4 wall following',
    entry_points={
        'console_scripts': [
            'odometry_logger_node = wf_odometry.odometry_logger_node:main',
        ],
    },
)

