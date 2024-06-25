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

		self.prev_angles = [-999, -999, -999, -999]

	def command_callback(self, msg):

		command = msg.data

		self.get_logger().info(f'Received command: {command}')

		try:
			parts = command.split(':')

			if len(parts) != 4:
				self.get_logger().info('Incorrect command : Are you sure you typed "angle_hl:angle_hr:angle_kl:angle_kr"')
				return
			
			# Selecting the angles and correcting them to respect the model I want
			angle_s1 = int(parts[0])
			angle_s2 = 180-int(parts[1])
			angle_s3 = int(parts[2])
			angle_s4 = 180-int(parts[3])

			angles = [angle_s1, angle_s2, angle_s3, angle_s4]
			pwms = [self.pwm1, self.pwm2, self.pwm3, self.pwm4]

			for e in parts:
				angle = int(e)
				if angle > 180 or angle < 0:
					self.get_logger().info('The angle you specified is not between 0 and 180 degrees')
					return
			
			for i in range(4):
				if angles[i] != self.prev_angles[i]:
					self.set_servo_angle(angles[i], pwms[i])
			
			self.prev_angles = angles


		except (IndexError, ValueError) as e:
			self.get_logger().info('Invalid command format. Please use : "<hl_angle>:<hr_angle>:<kl_angle>:<kr_angle>"')


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
		super().destroy_node()

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
