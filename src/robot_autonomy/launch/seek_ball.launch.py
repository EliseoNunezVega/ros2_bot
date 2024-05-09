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

    detect_ball = Node(
        package="get_ball_coordinates",
        executable="detect",
        parameters=[],
    )

    detect_ball_threed = Node(
        package="get_ball_coordinates",
        executable="detect3d",
        parameters=[],
    )

    autonomy_node_cmd = Node(
      package="robot_autonomy",
      executable="autonomy_node",
      name="autonomy_node",
    #   parameters=[{
    #       "location_file": os.path.join(pkg_tb3_sim, "config", "sim_house_locations.yaml")
    #   }]
    )

    return LaunchDescription([
        find_ball_server,
        go_to_ball_server,
        detect_ball,
        detect_ball_threed,
        autonomy_node_cmd
        # twist_stamper
    ])