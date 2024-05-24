import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math
import time

class MPU6050Publisher(Node):
    def __init__(self):
        super().__init__('mpu6050_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.sensor = mpu6050(0x68)
        self.timer = self.create_timer(0.1, self.publish_imu_data)
        
        self.last_time = time.time()
        self.alpha = 0.9
        
        # Initial orientation angles
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def publish_imu_data(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        imu_msg = Imu()

        # Read accelerometer data
        accel_data = self.sensor.get_accel_data()
        ax = accel_data['x']
        ay = accel_data['y']
        az = accel_data['z']

        # Read gyroscope data
        gyro_data = self.sensor.get_gyro_data()
        gx = math.radians(gyro_data['x'])
        gy = math.radians(gyro_data['y'])
        gz = math.radians(gyro_data['z'])

        # Calculate accelerometer angles
        accel_roll = math.atan2(ay, az) * 180 / math.pi
        accel_pitch = math.atan2(ax, az) * 180 / math.pi

        imu_msg.orientation.x = accel_roll
        imu_msg.orientation.y = accel_pitch


        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
