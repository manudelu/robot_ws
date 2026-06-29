from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(get_package_share_directory("robot_controller"), "config", "joy_config.yaml")
        ]
    )

    joy_teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        parameters=[{'enable_button': 7,
                    'axis_linear.x': 1,
                    'axis_angular.yaw': 0,
                    'scale_linear.x': 1.0,
                    'scale_angular.yaw': 1.0,
                    'publish_stamped_twist': True 
        }],
        remappings=[
            ('cmd_vel', '/robot_controller/cmd_vel')
        ]
    )

    return LaunchDescription([
        joy_node,
        joy_teleop
    ])