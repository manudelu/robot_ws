from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    imu_driver_node = Node(
        package="robot_firmware",
        executable="mpu6050_driver",
        output="screen"
    )

    return LaunchDescription(
        [
            imu_driver_node,
        ]
    )