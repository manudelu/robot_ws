import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("robot_description"), "launch", "gazebo.launch.py")
    )

    controller = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("robot_controller"), "launch", "controller.launch.py")
    )

    joystick = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("robot_controller"), "launch", "joystick_teleop.launch.py")
    )

    return LaunchDescription([
        gazebo, 
        controller,
        joystick
    ])