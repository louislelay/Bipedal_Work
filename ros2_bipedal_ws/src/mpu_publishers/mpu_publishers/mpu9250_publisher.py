import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

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
            gfs=GFS_250,   # 250 degrees per second
            afs=AFS_2G,    # 2G
            mfs=AK8963_BIT_16,  # 16-bit resolution
            mode=AK8963_MODE_C100HZ)

        self.mpu.configure()  # Apply the settings to the registers.

    def timer_callback(self):
        data = self.mpu.getAllData()
        msg = String()
        msg.data = f"Accel: {data[1]}, Gyro: {data[0]}, Mag: {data[2]}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
