#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import pigpio

class ServoInterface(Node):
    def __init__(self):
        super().__init__('servo_interface')
        self.pi = pigpio.pi()
        self.servo_pin_1 = 18  # GPIO pin for servo 1
        self.servo_pin_2 = 19  # GPIO pin for servo 2
        self.create_subscription(Float64, 'servo1_angle', self.servo1_callback, 10)
        self.create_subscription(Float64, 'servo2_angle', self.servo2_callback, 10)

    def set_servo_angle(self, pin, angle):
        pulse_width = 500 + (angle / 180.0) * 2000
        self.pi.set_servo_pulsewidth(pin, pulse_width)

    def servo1_callback(self, msg):
        self.get_logger().info(f'Setting servo 1 angle to {msg.data}')
        self.set_servo_angle(self.servo_pin_1, msg.data)

    def servo2_callback(self, msg):
        self.get_logger().info(f'Setting servo 2 angle to {msg.data}')
        self.set_servo_angle(self.servo_pin_2, msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ServoInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
