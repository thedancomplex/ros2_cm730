import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='cm730driver', node_executable='cm730driver_node',
             output='screen'),
        Node(package='cm730controller', node_executable='cm730controller_node',
             output='screen'),
        Node(package='mx_joint_state_publisher', node_executable='mx_joint_state_publisher_node',
             output='screen')
])
