import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

import numpy as np
import time
import math

class MPU9250Publisher(Node):
	def __init__(self):
		super().__init__('mpu9250_publisher')
		self.publisher_ = self.create_publisher(String, 'mpu9250_data', 10)
		timer_period = 1.0  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
		self.mpu = MPU9250(
			address_ak=AK8963_ADDRESS, 
			address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
			address_mpu_slave=None, 
			bus=1,
			gfs=GFS_1000, 
			afs=AFS_8G, 
			mfs=AK8963_BIT_16, 
			mode=AK8963_MODE_C100HZ)

		self.mpu.configure()  # Apply the settings to the registers.

		self.pitch = 0
		self.roll = 0

		self.last_time = time.time()

		
	def timer_callback(self):
		# Read the accelerometer, gyroscope, and magnetometer uncalibrated values
		self.accel_data = self.mpu.readAccelerometerMaster()
		self.gyro_data = self.mpu.readGyroscopeMaster()
		self.mag_data = self.mpu.readMagnetometerMaster()

		self.filtered_roll()

		msg = String()

		msg.data = str(self.pitch + self.roll)

		self.publisher_.publish(msg)

		self.get_logger().info('Publishing: \n"%s"' % msg.data)

	def filtered_pitch(self):
		GyrYd = self.gyro_data[1] / 131
		GyrYd = float(GyrYd) / 100
		pitchGyr = float(self.pitch - GyrYd)
		pitchAcc = float(180/3.141592)*math.atan2(self.accel_data[0], self.accel_data[2])

			# Calculate roll (rotation around the x-axis)
		self.roll = np.arctan2(ay, az)
		self.roll = np.degrees(self.roll)

		self.pitch = 0.9 * pitchGyr + 0.1 * pitchAcc


def main(args=None):
	rclpy.init(args=args)
	node = MPU9250Publisher()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
