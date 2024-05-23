import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

from mpu6050 import mpu6050
import math


class MPU6050Publisher(Node):

	def __init__(self):
		super().__init__('mpu6050_publisher')
		self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
		self.sensor = mpu6050(0x68)
		timer_period = 0.1  # seconds
		self.timer = self.create_timer(timer_period, self.publish_imu_data)
		self.i = 0

	def publish_imu_data(self):
		imu_msg = Imu()

		# Read accelerometer data
		accel_data = self.sensor.get_accel_data()
		imu_msg.linear_acceleration.x = accel_data['ax']
		imu_msg.linear_acceleration.y = accel_data['ay']
		imu_msg.linear_acceleration.z = accel_data['az']

		# Read gyroscope data
		gyro_data = self.sensor.get_gyro_data()
		imu_msg.angular_velocity.x = gyro_data['gx']
		imu_msg.angular_velocity.y = gyro_data['gy']
		imu_msg.angular_velocity.z = gyro_data['gz']

		# Publish the IMU data
		self.publisher_.publish(imu_msg)


def main(args=None):
	rclpy.init(args=args)
	mpu6050_publisher = MPU6050Publisher()
	rclpy.spin(mpu6050_publisher)
	mpu6050_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()