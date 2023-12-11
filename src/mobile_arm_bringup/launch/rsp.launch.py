# $LICENSE$

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="mobile_arm_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="main.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    

    # Get URDF via xacro
    # xacro_file = os.path.join(get_package_share_directory(description_package), 'urdf', description_file)
    xacro_file = PathJoinSubstitution([FindPackageShare(description_package), 'description/pioneer3dx', description_file])
    robot_description = {
        'robot_description': Command(['xacro ', xacro_file, " use_sim:=false"])
    }

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{'use_sim_time': True}],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
        ]
    )