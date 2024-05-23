import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Subscribe to IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        
        # Publisher for servo angles
        self.angle_publisher = self.create_publisher(
            String,
            'angle_desired',
            10)
        
        # PID controller parameters
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        
        self.previous_error = 0.0
        self.integral = 0.0
        
    def imu_callback(self, msg):
        # Get the roll angle from IMU data
        roll = imu_msg.orientation.x
        
        # Desired roll angle (straight)
        desired_roll = 0.0
        
        # Compute error
        error = desired_roll - roll
        
        # PID control
        self.integral += error
        derivative = error - self.previous_error
        
        control_signal = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.previous_error = error
        
        # Publish the control signal as the desired angle for the servos
        angle_msg = Float64()
        angle_msg.data = control_signal
        self.angle_publisher.publish(angle_msg)

def main(args=None):
    rclpy.init(args=args)
    
    servo_controller = ServoController()
    
    rclpy.spin(servo_controller)
    
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
