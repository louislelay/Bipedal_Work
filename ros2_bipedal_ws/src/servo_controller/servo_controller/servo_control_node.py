import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class ServoController(Node):

    def __init__(self):
        super().__init__('servo_controller')
        self.subscription = self.create_subscription(
            Float32,
            'angle_hip_left',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # GPIO setup
        self.servo_pin = 18  # Change this to your GPIO pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)  # 50Hz frequency
        self.pwm.start(0)

    def listener_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f'Received angle: {angle}')
        self.set_servo_angle(angle)

    def set_servo_angle(self, angle):
        duty_cycle = self.angle_to_duty_cycle(angle)
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)  # Allow time for the servo to reach the position
        self.pwm.ChangeDutyCycle(0)  # Stop sending signals to the servo

    def angle_to_duty_cycle(self, angle):
        # Convert angle (0-180) to duty cycle (2-12)
        return 2 + (angle / 18)

    def destroy(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    try:
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        pass
    finally:
        servo_controller.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
