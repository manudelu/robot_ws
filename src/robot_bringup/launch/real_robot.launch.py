import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    hardware_interface = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("robot_firmware"), "launch", "hardware_interfaces.launch.py")
    )

    controller = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("robot_controller"), "launch", "controller.launch.py")
    )

    joystick = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("robot_controller"), "launch", "joystick_teleop.launch.py")
    )

    imu_driver_node = Node(
        package="robot_firmware",
        executable="mpu6050_driver"
    )

    lidar = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_composition",
        parameters=[{
            'serial_port': "/dev/ttyUSB0",
            'channel_type': "serial",
            'serial_baudrate': 115200,
            'frame_id': "laser_link",
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': "Sensitivity"
        }],
    )

    return LaunchDescription([
        hardware_interface, 
        controller,
        joystick,
        imu_driver_node,
        #lidar
    ])