import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from action_bipedal_interface.action import DcController
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
            100)
        
        # Publisher for servo angles
        self.angle_publisher = self.create_publisher(
            String,
            'servo_command',
            10)

        # Action client for DC controller
        self._action_client = ActionClient(self, DcController, 'dc_controller')

        # PID controller parameters
        self.I = 0.
        self.prev_input = 0.

        # Publish the control signal as the desired angle for the servos
        servo_signal = '40:40:0:10'
        command_msg = String()
        command_msg.data = servo_signal
        self.angle_publisher.publish(command_msg)
        
    def imu_callback(self, msg):
        # Get the roll angle from IMU data
        roll = int(float(msg.data))

        vit_mot = int(self.PID(roll))
        
        # Send command to DCControllerActionServer
        self.send_goal(vit_mot)

    def send_goal(self, speed):
        goal_msg = DcController.Goal()
        goal_msg.goal = speed

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Final RPM: {result.final_rpm}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_rpm}')

    def PID(self, input):
        Kp = 10
        Ki = 0
        Kd = 2

        P = Kp * float(input)
        self.I += Ki * float(input)
        D = Kd * (input - self.prev_input)
        
        self.prev_input = input
        
        PID = int(P + self.I + D)

        if PID > 100:
            PID = 100
        elif PID < -100:
            PID = -100

        return PID

def main(args=None):
    rclpy.init(args=args)
    
    balance = Balance()
    
    rclpy.spin(balance)
    
    balance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
