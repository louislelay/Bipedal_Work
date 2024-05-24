import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

import numpy as np
import time

class MPU9250Publisher(Node):
	def __init__(self):
		super().__init__('mpu9250_publisher')
		self.publisher_ = self.create_publisher(String, 'mpu9250_data', 10)
		timer_period = 1.0  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
		self.mpu = MPU9250(
			address_ak=AK8963_ADDRESS,
			address_mpu_master=MPU9050_ADDRESS_68,
			address_mpu_slave=None,
			bus=1,
			gfs=GFS_1000,   # 1000 degrees per second
			afs=AFS_8G,    # 8G
			mfs=AK8963_BIT_16,  # 16-bit resolution
			mode=AK8963_MODE_C100HZ)

		self.mpu.configure()  # Apply the settings to the registers.

		self.callibrate_gyro()
		self.callibrate_accel()

		self.angle = [0, 0, 0]

		self.last_time = time.time()

		
	def timer_callback(self):
		# Read the accelerometer, gyroscope, and magnetometer uncalibrated values
		accel_data = self.mpu.readAccelerometerMaster()
		gyro_data = self.mpu.readGyroscopeMaster()
		mag_data = self.mpu.readMagnetometerMaster()

		# Apply the calibration offsets 
		self.gyro_data[0] -= self.gx_offset
		self.gyro_data[1] -= self.gy_offset
		self.gyro_data[2] -= self.gz_offset

		self.accel_data[0] -= self.ax_offset
		self.accel_data[1] -= self.ay_offset
		self.accel_data[2] -= self.az_offset

		self.calculate_angle()

		msg = String()
		msg.data = f"Angle: {self.angle}\n\n"
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing: \n"%s"' % msg.data)

		self.num_samples=1000
	
	def callibrate_gyro(self):
		print("Calibrating gyroscope...")
		print("Please keep the sensor stationary during calibration.")
		
		gyro_data = []
		for _ in range(self.num_samples):
			gyro_data.append(self.mpu.readGyroscopeMaster())
			time.sleep(0.01)
		
		gyro_data = np.array(gyro_data)
		self.gx_offset, self.gy_offset, self.gz_offset = np.mean(gyro_data, axis=0)
		
		print("Calibration complete.")
		print("Gyroscope offsets: gx_offset={}, gy_offset={}, gz_offset={}".format(gx_offset, gy_offset, gz_offset))

	
	def callibrate_accel(self):
		print("Calibrating accelerometer...")
		print("Please follow the instructions for each axis.")
		
		self.accel_offsets = [0, 0, 0]
		axis_labels = ['x', 'y', 'z']
		
		for axis in range(3):
			print("Orient the sensor so that the {} axis is pointed against gravity.".format(axis_labels[axis]))
			input("Press Enter when ready...")
			
			accel_data = []
			for _ in range(self.num_samples):
				accel_data.append(mpu.readAccelerometerMaster()[axis])
				time.sleep(0.01)
			
			accel_offset = np.mean(accel_data)
			self.accel_offsets[axis] = accel_offset - 1
			
			print("{} axis offset: {}".format(axis_labels[axis], self.accel_offsets[axis]))
		
			print("Calibration complete.")
			print("Accelerometer offsets: ax_offset={}, ay_offset={}, az_offset={}".format(*self.accel_offsets))
	
	def calculate_angle(self):
		current_time = time.time()
		dt = current_time - self.last_time
		self.last_time = current_time
		# Using Complementary Filter
		A = 0.98
		
		for i in range(3):
			self.angle[i] = A * (self.angle[i]+self.gyro_data[i]*dt) + (1-A) * self.accel_data[i]

def main(args=None):
	rclpy.init(args=args)
	node = MPU9250Publisher()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
