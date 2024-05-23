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
        self.alpha = 0.98
        
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
        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Complementary filter to combine gyroscope and accelerometer data
        self.roll = self.alpha * (self.roll + gx * dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * accel_pitch

        # Integrate gyroscope z-axis data for yaw
        self.yaw += gz * dt

        # Convert roll, pitch, yaw to quaternion
        q = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.angular_velocity.x = gyro_data['x']
        imu_msg.angular_velocity.y = gyro_data['y']
        imu_msg.angular_velocity.z = gyro_data['z']

        self.publisher_.publish(imu_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
