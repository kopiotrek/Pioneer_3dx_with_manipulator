import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, PathJoinSubstitution

import xacro


def generate_launch_description():

    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim = LaunchConfiguration('use_sim')
    
    xacro_file = PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file])

    robot_description_config = Command(
        ['xacro ', xacro_file, 'use_sim:=', use_sim])

    params = {'robot_description': robot_description_config,
              'use_sim_time': use_sim_time}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation'),

        node_robot_state_publisher
    ])
