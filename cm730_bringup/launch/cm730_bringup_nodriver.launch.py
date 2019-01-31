import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='cm730controller', node_executable='cm730controller_node',
             output='screen'),
        Node(package='mx_joint_controller', node_executable='mx_joint_controller_node',
             output='screen')
])
