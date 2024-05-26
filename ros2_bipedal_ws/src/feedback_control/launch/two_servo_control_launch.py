from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpu_publishers',
            executable='mpu9250_publisher',
            name='mpu9250_publisher'
        ),
        Node(
            package='servo_controller',
            executable='servo_controller_node',
            name='servo_controller_node'
        ),
        Node(
            package='feedback_control',
            executable='two_servo_controller',
            name='two_servo_controller'
        ),
    ])
