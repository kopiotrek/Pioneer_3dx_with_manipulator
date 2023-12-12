import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ariadna_simulation_bringup_package = get_package_share_directory('ariadna_simulation_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_manipulator',
            default_value='True',
            description='Loads manipulator description and controllers'),

        DeclareLaunchArgument(
            name='world',
            default_value=os.path.join(
                ariadna_simulation_bringup_package, 'worlds', 'willowgarage.world'),
            description='Full path to the world model file to load'),

        DeclareLaunchArgument(
            name='gui',
            default_value='true',
            description='Set to "false" to run headless.'
        ),

        DeclareLaunchArgument(
            name='server',
            default_value='true',
            description='Set to "false" not to run gzserver.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ariadna_simulation_bringup_package, '/launch/gazebo.launch.py']),
            launch_arguments={'use_manipulator': LaunchConfiguration('use_manipulator')}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ariadna_simulation_bringup_package, '/launch/spawn_ariadna.launch.py']),
        )
    ])
