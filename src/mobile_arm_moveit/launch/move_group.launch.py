import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declare_description_package_arg = DeclareLaunchArgument(
            "description_package",
            default_value="cyton_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
    )
    declare_description_file_arg = DeclareLaunchArgument(
            "description_file",
            default_value="cyton.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
    )
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Start robot in Gazebo simulation.",
    )

    use_sim = LaunchConfiguration("use_sim")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    moveit_config = MoveItConfigsBuilder("cyton", package_name="cyton_moveit").to_moveit_configs()
    xacro_file = PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file])
    robot_description_content = {
        'robot_description': Command(['xacro ', xacro_file, " use_sim:=", use_sim])
    }

    moveit_config.robot_description = {"robot_description": robot_description_content}
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "monitor_dynamics": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.0,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params
    )

    actions = [
        declare_use_sim_arg,
        declare_description_package_arg,
        declare_description_file_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        move_group_node,
    ]
    return LaunchDescription(actions)
