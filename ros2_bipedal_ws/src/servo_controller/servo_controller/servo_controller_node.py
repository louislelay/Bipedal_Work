import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class ServoController(Node):

	def __init__(self):
		super().__init__('servo_controller')
		self.subscription = self.create_subscription(
			String, # type of joint (hl, hr, kl, kr) and desired angle 
			'servo_command',
			self.command_callback,
			10)
		self.subscription  # prevent unused variable warning


		# GPIO setup

		self.servo_pins = [17, 18, 27, 22]  # Change this to your GPIO pins

		GPIO.setmode(GPIO.BCM)

		for pin in self.servo_pins:
			GPIO.setup(pin, GPIO.OUT)

		self.pwm1 = GPIO.PWM(self.servo_pins[0], 50)  # 50Hz frequency
		self.pwm1.start(0)

		self.pwm2 = GPIO.PWM(self.servo_pins[1], 50)  # 50Hz frequency
		self.pwm2.start(0)

		self.pwm3 = GPIO.PWM(self.servo_pins[2], 50)  # 50Hz frequency
		self.pwm3.start(0)

		self.pwm4 = GPIO.PWM(self.servo_pins[3], 50)  # 50Hz frequency
		self.pwm4.start(0)

	def command_callback(self, msg):

		command = msg.data

		self.get_logger().info(f'Received command: {command}')

		if command.startswith('hl'):
			pwm = self.pwm1
			parts = command.split(':')
			angle = int(parts[1])
		elif command.startswith('hr'):
			pwm = self.pwm1
			parts = command.split(':')
			angle = int(parts[1])
		elif command.startswith('kl'):
			pwm = self.pwm1
			parts = command.split(':')
			angle = int(parts[1])
		elif command.startswith('kr'):
			pwm = self.pwm1
			parts = command.split(':')
			angle = int(parts[1])
		else:
			print('Invalid command, please use : "<hl/hr/kl/kr>:<0 to 180>"')
			pwm = None
			angle = 0
		
		if angle > 180 or angle < 0:
			print('The angle you specified is not between 0 and 180 degrees')
		
		set_servo_angle(angle, pwm)

	def set_servo_angle(self, angle, pwm):
		duty_cycle = self.angle_to_duty_cycle(angle)
		pwm.ChangeDutyCycle(duty_cycle)
		time.sleep(0.5)  # Allow time for the servo to reach the position
		pwm.ChangeDutyCycle(0)  # Stop sending signals to the servo

	def angle_to_duty_cycle(self, angle):
		# Convert angle (0-180) to duty cycle (2-12)
		return 2 + (angle / 18)

	def destroy(self):
		self.pwm1.stop()
		self.pwm2.stop()
		self.pwm3.stop()
		self.pwm4.stop()
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
