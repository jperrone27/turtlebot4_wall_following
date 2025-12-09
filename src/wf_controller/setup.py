from setuptools import find_packages, setup

package_name = 'wf_controller'

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
    description='Wall following controller for TurtleBot 4',
    entry_points={
        'console_scripts': [
            'wall_controller_node = wf_controller.wall_controller_node:main',
        ],
    },
)
