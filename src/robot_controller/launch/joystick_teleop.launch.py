from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[{
            'device_id': 0,
            'device_name': "",
            'deadzone': 0.5,
            'autorepeat_rate': 20.0,
            'sticky_button': False,
            'coalesce_interval_ms': 1
        }]
    )

    joy_teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        parameters=[{
            'enable_button': 7,
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