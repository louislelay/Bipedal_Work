import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import time

class Balance(Node):
	def __init__(self):
		super().__init__('balance')
		
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

		# Publisher for speed of DC
		self.dc_publisher = self.create_publisher(
			String,
			'dc_command',
			10)
		
		# PID controller parameters
		self.I = 0.
		self.prev_input = 0.

		# Publish the control signal as the desired angle for the servos
		servo_signal = '90:90:90:90'		# 40:40:0:10
		command_msg = String()
		command_msg.data = servo_signal
		self.angle_publisher.publish(command_msg)
		
	def imu_callback(self, msg):

		t = time.time()
		t_next = t+10

		# Get the roll angle from IMU data
		roll = msg.data

		vit_mot = int(self.PID(roll))
		print("vit_mot : "+ vit_mot)

		t_real = time.time()
		time.sleep(t_next-t_real)

		
		dc_signal = str(vit_mot)
		command_msg = String()
		command_msg.data = dc_signal
		self.dc_publisher.publish(command_msg)

	def PID(self, input):
		Kp = 250.
		Ki = 0.
		Kd = 0

		P = Kp * float(input)
		I += Ki * float(input)
		D = Kd * (input - self.prev_input)
		
		self.prev_input = input
		
		PID = int(P+I+D)

		if (PID>100): PID = 100
		if (PID<-100): PID = -100

		return PID



		

def main(args=None):
	rclpy.init(args=args)
	
	balance = Balance()
	
	rclpy.spin(balance)
	
	balance.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
