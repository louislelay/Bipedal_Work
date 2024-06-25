import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class DCController(Node):

	def __init__(self):
		super().__init__('dc_controller')
		self.subscription = self.create_subscription(
			String, # type of joint (wl, wr) and desired speed
			'dc_command',
			self.command_callback,
		    70)
		self.subscription  # prevent unused variable warning


		# GPIO setup

		self.ENC_A1 = 27  # GPIO pin for encoder 1 of Motor A
		self.ENC_A2 = 17  # GPIO pin for encoder 2 of Motor A
		self.EN_A = 22         # GPIO pin for L298N enable of Motor A
		self.IN_1 = 26         # GPIO pin for L298N direction of Motor A
		self.IN_2 = 6          # GPIO pin for L298N direction of Motor A

		self.EN_B = 5         # GPIO pin for L298N enable of Motor A
		self.IN_3 = 12         # GPIO pin for L298N direction of Motor A
		self.IN_4 = 16          # GPIO pin for L298N direction of Motor A

		# Variables for encoder
		self.enc_A1_last = GPIO.LOW
		self.duration = 0
		self.abs_duration = 0
		self.direction = True  # Default direction forward

		# PID constants
		self.kp = 0.6
		self.ki = 5
		self.kd = 0

		# PID variables
		self.setpoint = 0	# Speed wanted
		self.output = 0
		self.prev_error = 0
		self.integral = 0
		self.last_time = time.time()

		# GPIO setup
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.EN_A, GPIO.OUT)
		GPIO.setup(self.IN_1, GPIO.OUT)
		GPIO.setup(self.IN_2, GPIO.OUT)

		GPIO.setup(self.EN_B, GPIO.OUT)
		GPIO.setup(self.IN_3, GPIO.OUT)
		GPIO.setup(self.IN_4, GPIO.OUT)

		GPIO.setup(self.ENC_A1, GPIO.IN)
		GPIO.setup(self.ENC_A2, GPIO.IN)

		# Initialize PWMA
		self.pwmA = GPIO.PWM(self.EN_A, 1000)  # Initialize PWM on E_LEFT pin 1000Hz frequency
		self.pwmA.start(0)

		# Initialize PWMB
		self.pwmB = GPIO.PWM(self.EN_B, 1000)  # Initialize PWM on E_LEFT pin 1000Hz frequency
		self.pwmB.start(0)

		# Initialize encoder
		#GPIO.add_event_detect(self.ENC_A1, GPIO.BOTH, callback=encoder_callback)

		

	def command_callback(self, msg):

		command = msg.data

		self.get_logger().info(f'Received command: {command}')

		self.setpoint = abs(int(command))



		self.abs_duration = abs(self.duration)
		self.compute_pid()  # Compute the PID output

		if int(command) > 0 : self.advance()
		else : self.back()

		print(f"Pulse: {self.duration}")
		self.duration = 0  # Reset duration for next count
		time.sleep(1/70)  # Sleep for 100ms

	def encoder_callback(self, channel):
		Lstate = GPIO.input(self.ENC_A1)
		if (self.enc_A1_last == GPIO.LOW) and Lstate == GPIO.HIGH:
			val = GPIO.input(self.ENC_A2)
			if val == GPIO.LOW and self.direction:
				self.direction = False  # Reverse
			elif val == GPIO.HIGH and not Direction:
				self.direction = True  # Forward

		self.enc_A1_last = Lstate

		if not self.direction:
			self.duration += 1
		else:
			self.duration -= 1


	def compute_pid(self):		
		current_time = time.time()

		dt = current_time - self.last_time
		
		error = self.setpoint - self.abs_duration
		self.integral += error * dt
		derivative = (error - self.prev_error) / dt if dt > 0 else 0
		
		self.output = self.kp * error + self.ki * self.integral + self.kd * derivative
		self.output = max(0, min(100, self.output))  # Clamp self.output to range 0-100
		
		self.prev_error = error
		self.last_time = current_time

	def advance(self):
		self.pwmA.ChangeDutyCycle(self.output)
		GPIO.output(self.IN_1, GPIO.HIGH)
		GPIO.output(self.IN_2, GPIO.LOW)

		self.pwmB.ChangeDutyCycle(self.output)
		GPIO.output(self.IN_3, GPIO.HIGH)
		GPIO.output(self.IN_4, GPIO.LOW)

	def back(self):
		self.pwmA.ChangeDutyCycle(self.output)
		GPIO.output(self.IN_1, GPIO.LOW)
		GPIO.output(self.IN_2, GPIO.HIGH)

		self.pwmB.ChangeDutyCycle(self.output)
		GPIO.output(self.IN_3, GPIO.LOW)
		GPIO.output(self.IN_4, GPIO.HIGH)

	def stop(self):
		self.pwmA.ChangeDutyCycle(0)
		GPIO.output(self.IN_1, GPIO.LOW)
		GPIO.output(self.IN_2, GPIO.LOW)

		self.pwmB.ChangeDutyCycle(0)
		GPIO.output(self.IN_3, GPIO.LOW)
		GPIO.output(self.IN_4, GPIO.LOW)
	
	def destroy(self):
		self.pwmA.stop()
		self.pwmB.stop()
		GPIO.cleanup()

def main(args=None):
	rclpy.init(args=args)
	dc_controller = DCController()
	try:
		rclpy.spin(dc_controller)
	except KeyboardInterrupt:
		pass
	finally:
		dc_controller.destroy()
		rclpy.shutdown()

if __name__ == '__main__':
	main()