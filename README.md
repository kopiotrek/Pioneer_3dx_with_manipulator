# Cyton300_ROS2

Running on ROS Humble

How to run:
1. docker-compose up --build --d
2. docker-compose exec cyton_service bash

Notes:

sudo chmod 777 /dev/ttyUSB0


Cyton consists of: 
    - 1 MX-64 dynamixel
    - 6 MX-28 dynamixels
    - 1 AX-12A dynamixels

# Test arm

ros2 launch cyton_moveit bajor_move_group.launch.py 

ros2 launch cyton_moveit bajor_moveit_rviz.launch.py 

ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7],
    points: [
        { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 5, nanosec: 0 } },
    ]
  }
}"

ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7],
    points: [
        { positions: [-0.8053399324417114, 0.21, -0.09357283264398575, -0.5798447728157043, 2.63077712059021, -1.7272623777389526, -0.4724660813808441], time_from_start: { sec: 5, nanosec: 0 } },
    ]
  }
}"

[-0.8053399324417114, 0.19328159093856812, -0.09357283264398575, -0.5798447728157043, 2.63077712059021, -1.7272623777389526, -0.4724660813808441, 0.015929520452224677]

# Test Pioneer:

ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

Issues:
Drgania - Nie można zapisać PID serw do uruchomienia, brak sterowania prędkością
Problem z zegarem symulacji



