from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_manipulator = LaunchConfiguration('use_manipulator')
    spawn_ariadna = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ariadna', '-topic', 'robot_description'],
        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start",
             "joint_state_broadcaster"],
        output="screen"
    )

    load_forward_command_controller = ExecuteProcess(
        cmd=['ros2', 'control',  "load_controller", "--set-state", "start",
             'forward_command_controller'],
        output='screen',
        condition=IfCondition(use_manipulator)
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start",
             "joint_trajectory_controller"],
        output="screen",
        condition=IfCondition(use_manipulator)
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start",
             "diff_drive_controller"],
        output="screen"
    )

    return LaunchDescription([
        spawn_ariadna,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_ariadna,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_diff_drive_controller,
                on_exit=[load_forward_command_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_forward_command_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        )
    ])
