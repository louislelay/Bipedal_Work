from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpu6050_publisher',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen'
        ),
        Node(
            package='servo_controller',
            executable='servo_controller_node',
            name='servo_controller_node',
            output='screen'
        ),
        Node(
            package='feedback_control',
            executable='2servo_controller',
            name='2servo_controller',
            output='screen'
        ),
    ])
