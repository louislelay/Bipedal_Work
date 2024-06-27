import smbus2
import time
import math
import numpy as np
import RPi.GPIO as GPIO

# MPU-9250 Registers and their addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

bus = smbus2.SMBus(1)
Device_Address = 0x68

# Initialize MPU-9250
def MPU_Init():
	# Write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	# Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	# Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	# Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	# Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	# Accelero and Gyro value are 16-bit
	high = bus.read_byte_data(Device_Address, addr)
	low = bus.read_byte_data(Device_Address, addr + 1)
	value = ((high << 8) | low)
	# to get signed value from mpu6050
	if value > 32768:
		value = value - 65536
	return value

# Servo Motor Setup
servo_pins = [25, 18, 24, 23]
GPIO.setmode(GPIO.BCM)
for pin in servo_pins:
	GPIO.setup(pin, GPIO.OUT)

servo_pwm = [GPIO.PWM(pin, 50) for pin in servo_pins]
for pwm in servo_pwm:
	pwm.start(0)

# DC Motor Setup
motor_pins = {'ENA': 22, 'IN1': 26, 'IN2': 6, 'ENB': 5, 'IN3': 12, 'IN4': 16}
for pin in motor_pins.values():
	GPIO.setup(pin, GPIO.OUT)

def set_servo_angle(pwm, angle):
	duty = angle / 18 + 2
	pwm.ChangeDutyCycle(duty)
	#time.sleep(0.5)
	#pwm.ChangeDutyCycle(0)

def control_dc_motor(motor_pins, speed, direction):
	if direction == 'forward':
		GPIO.output(motor_pins['IN1'], GPIO.HIGH)
		GPIO.output(motor_pins['IN2'], GPIO.LOW)
	elif direction == 'backward':
		GPIO.output(motor_pins['IN1'], GPIO.LOW)
		GPIO.output(motor_pins['IN2'], GPIO.HIGH)
	else:
		GPIO.output(motor_pins['IN1'], GPIO.LOW)
		GPIO.output(motor_pins['IN2'], GPIO.LOW)

	pwm = GPIO.PWM(motor_pins['ENA'], 1000)
	pwm.start(speed)

# Main loop
MPU_Init()

try:
	while True:
		# Read Accelerometer raw value
		acc_x = read_raw_data(ACCEL_XOUT_H)
		acc_y = read_raw_data(ACCEL_YOUT_H)
		acc_z = read_raw_data(ACCEL_ZOUT_H)

		# Read Gyroscope raw value
		gyro_x = read_raw_data(GYRO_XOUT_H)
		gyro_y = read_raw_data(GYRO_YOUT_H)
		gyro_z = read_raw_data(GYRO_ZOUT_H)

		# Full scale range +/- 250 degree/C as per sensitivity scale factor
		Ax = acc_x / 16384.0
		Ay = acc_y / 16384.0
		Az = acc_z / 16384.0

		Gx = gyro_x / 131.0
		Gy = gyro_y / 131.0
		Gz = gyro_z / 131.0

		# Balancing logic
		angle_x = np.arctan2(Ay, Az)
		angle_x = np.degrees(angle_x)
		print(angle_x)

		angless = [40, 180-40, 0, 180-10]

		# Set servo positions based on angles
		for i in range(4):
			set_servo_angle(servo_pwm[i], angless[i])

		# Control DC motors based on tilt
		if angle_x > 5:
			control_dc_motor(motor_pins, 50, 'forward')
		elif angle_x < -5:
			control_dc_motor(motor_pins, 50, 'backward')
		else:
			control_dc_motor(motor_pins, 0, 'stop')

		time.sleep(0.001)

except KeyboardInterrupt:
	pass

finally:
	for pwm in servo_pwm:
		pwm.stop()
	GPIO.cleanup()
