from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='wf_perception',
             executable='wall_perception_node',
             output='screen'),
        Node(package='wf_controller',
             executable='wall_controller_node',
             output='screen'),
        Node(package='wf_odometry',
             executable='odometry_logger_node',
             output='screen'),
    ])
