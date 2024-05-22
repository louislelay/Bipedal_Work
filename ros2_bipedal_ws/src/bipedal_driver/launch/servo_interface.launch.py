from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bipedal_driver',
            executable='servo_interface',
            name='servo_interface',
            output='screen'
        )
    ])