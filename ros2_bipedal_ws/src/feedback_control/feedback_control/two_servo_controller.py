import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

class ServoController(Node):
	def __init__(self):
		super().__init__('servo_controller')
		
		# Subscribe to IMU data
		self.imu_subscription = self.create_subscription(
			String,
			'mpu9250_data',
			self.imu_callback,
			10)
		
		# Publisher for servo angles
		self.angle_publisher = self.create_publisher(
			String,
			'servo_command',
			10)
		
		# PID controller parameters
		self.kp = 1.0
		self.ki = 0.0
		self.kd = 0.0
		
		self.previous_error = 0.0
		self.integral = 0.0
		
	def imu_callback(self, msg):
		# Get the roll angle from IMU data
		angles = msg.data

		parts = angles.split(':')

		roll = int(float(parts[0]))

		inital_angle_s = 90

		desired_angle = inital_angle_s - roll

		angle = int(desired_angle)
		control_signal = str(angle) + ":" + str(angle) +":0:0"

		# Publish the control signal as the desired angle for the servos
		command_msg = String()
		command_msg.data = control_signal
		self.angle_publisher.publish(command_msg)

def main(args=None):
	rclpy.init(args=args)
	
	servo_controller = ServoController()
	
	rclpy.spin(servo_controller)
	
	servo_controller.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
