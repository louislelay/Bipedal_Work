# servo_controller.py
import rclpy
from rclpy.node import Node
import pigpio

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.pi = pigpio.pi()

        # Define the PWM pins for the servos
        self.servo_pins = [17, 18, 27, 22]  # Example GPIO pins

        # Initialize servos
        for pin in self.servo_pins:
            self.pi.set_mode(pin, pigpio.OUTPUT)

        # Create a timer to periodically call the control function
        self.timer = self.create_timer(0.1, self.control_servos)

    def control_servos(self):
        # Example control: Set servos to a specific pulse width
        for pin in self.servo_pins:
            self.pi.set_servo_pulsewidth(pin, 1500)  # Set all servos to neutral position (1.5ms)

    def stop_servos(self):
        for pin in self.servo_pins:
            self.pi.set_servo_pulsewidth(pin, 0)

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    finally:
        node.stop_servos()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
