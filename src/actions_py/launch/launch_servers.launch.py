from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'actions_py'

    find_ball_server = Node(
            package= package_name,
            executable= 'find_ball_server',
            parameters=[],
         )

    go_to_ball_server = Node(
            package=package_name,
            executable='go_to_ball_server',
            parameters=[],
         )

    return LaunchDescription([
        find_ball_server,
        go_to_ball_server,
        # twist_stamper
    ])