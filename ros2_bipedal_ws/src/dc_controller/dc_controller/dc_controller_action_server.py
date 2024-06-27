import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_bipedal_interface.action import DcController

import RPi.GPIO as GPIO
import time


class DCControllerActionServer(Node):

	def __init__(self):
		super().__init__('dc_controller_action_server')
		self._action_server = ActionServer(
			self,
			DcController,
			'dc_controller',
			self.execute_callback)
		
		# GPIO setup
		self.EN_A = 22         # GPIO pin for L298N enable of Motor A
		self.IN_1 = 26         # GPIO pin for L298N direction of Motor A
		self.IN_2 = 6          # GPIO pin for L298N direction of Motor A

		self.EN_B = 5         # GPIO pin for L298N enable of Motor A
		self.IN_3 = 12         # GPIO pin for L298N direction of Motor A
		self.IN_4 = 16          # GPIO pin for L298N direction of Motor A

		self.ENC_A1 = 27  # GPIO pin for encoder 1 of Motor A
		self.ENC_A2 = 17  # GPIO pin for encoder 2 of Motor A

		# PID constants
		self.kp = 5
		self.ki = 0.6
		self.kd = 0

		# Variables for encoder
		self.counts_per_rev = 1942
		self.counter = 0
		self.rpm = 0
		self.last_time_enc = time.time()

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

		GPIO.setup(self.ENC_A1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.setup(self.ENC_A2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

		self.last_state_a1 = GPIO.input(self.ENC_A1)

		# Initialize PWMA
		self.pwmA = GPIO.PWM(self.EN_A, 1000)  # Initialize PWM on E_LEFT pin 1000Hz frequency
		self.pwmA.start(0)

		# Initialize PWMB
		self.pwmB = GPIO.PWM(self.EN_B, 1000)  # Initialize PWM on E_LEFT pin 1000Hz frequency
		self.pwmB.start(0)

		# Initialize encoder
		GPIO.add_event_detect(self.ENC_A1, GPIO.BOTH, callback=self.encoder_callback)
		

	def encoder_callback(self, channel):
		state_a1 = GPIO.input(self.ENC_A1)
		state_a2 = GPIO.input(self.ENC_A2)

		if state_a1 != self.last_state_a1:  # A has changed
			if state_a1 == state_a2:
				self.counter += 1
			else:
				self.counter -= 1

		self.last_state_a1 = state_a1
		
		#print(f"Counter: {self.counter}")

	def calculate_rpm(self):
		current_time = time.time()
		elapsed_time = current_time - self.last_time_enc
		print(f"elapsed time: {elapsed_time}")
		self.last_time_enc = current_time

		# Calculate RPM
		revolutions = self.counter / self.counts_per_rev
		print(f"counter: {self.counter}")
		self.counter = 0  # Reset counter after calculating RPM
		self.rpm = (revolutions / elapsed_time)*60

	def compute_pid(self):		
		current_time = time.time()

		dt = current_time - self.last_time

		self.abs_rpm = abs(self.rpm)
		
		error = self.setpoint - self.abs_rpm
		self.integral += error * dt
		derivative = (error - self.prev_error) / dt if dt > 0 else 0
		
		self.output = self.kp * error + self.ki * self.integral + self.kd * derivative
		self.output = max(0, min(100, self.output))  # Clamp self.output to range 0-100
		print(f"output: {self.output}")
		
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

	def execute_callback(self, goal_handle):
		# PID variables
		self.integral = 0

		self.get_logger().info('Executing goal...')

		feedback_msg = DcController.Feedback()
		feedback_msg.output = self.output

		command = goal_handle.request.setpoint
		self.setpoint = abs(command)

		while self.setpoint != self.output :

			self.calculate_rpm()
			print(f"Motor RPM: {self.rpm}")

			self.compute_pid()  # Compute the PID output

			if int(command) > 0 : self.advance()
			else : self.back()

			feedback_msg.rpm = self.rpm
			self.get_logger().info('Feedback: {0}'.format(feedback_msg.rpm))
			goal_handle.publish_feedback(feedback_msg)

		result = DcController.Result()
		result.rpm = feedback_msg.output
		return result


def main(args=None):
	rclpy.init(args=args)

	dc_controller_action_server = DCControllerActionServer()

	rclpy.spin(dc_controller_action_server)

if __name__ == '__main__':
	main()