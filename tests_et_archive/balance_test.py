import smbus2
import time
import math
import numpy as np
import RPi.GPIO as GPIO
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# MPU-9250 Registers and their addresses
mpu = MPU9250(
	address_ak=AK8963_ADDRESS, 
	address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
	address_mpu_slave=None, 
	bus=1,
	gfs=GFS_1000, 
	afs=AFS_8G, 
	mfs=AK8963_BIT_16, 
	mode=AK8963_MODE_C100HZ)
	

# Initialize MPU-9250
def MPU_Init():
	mpu.configure()  # Apply the settings to the registers.

	pitch = 0
	roll = 0

	last_time = time.time()

def read_raw_data():
	accel_data = mpu.readAccelerometerMaster()
	gyro_data = mpu.readGyroscopeMaster()
	mag_data = mpu.readMagnetometerMaster()

	return filtered_pitch_roll(accel_data)

def filtered_pitch_roll(accel_data):
	#GyrYd = gyro_data[1] / 131
	#GyrYd = float(GyrYd) / 100
	#pitchGyr = float(pitch - GyrYd)
	#pitchAcc = float(180/3.141592)*math.atan2(accel_data[0], accel_data[2])

	# Calculate roll (rotation around the x-axis)
	roll = np.arctan2(accel_data[1], accel_data[2])
	roll = np.degrees(roll)

	return roll
	#pitch = 0.9 * pitchGyr + 0.1 * pitchAcc

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

# Initialize PWMA
pwmA = GPIO.PWM(22, 1000)  # Initialize PWM on E_LEFT pin 1000Hz frequency
pwmA.start(0)

# Initialize PWMB
pwmB = GPIO.PWM(5, 1000)  # Initialize PWM on E_LEFT pin 1000Hz frequency
pwmB.start(0)

# PID controller parameters
I = 0.
prev_input = 0.

def set_servo_angle(pwm, angle):
	duty = angle / 18 + 2
	pwm.ChangeDutyCycle(duty)
	#time.sleep(0.5)
	#pwm.ChangeDutyCycle(0)

def PID(input, I, prev_input):
	Kp = 20
	Ki = 10
	Kd = 10

	P = Kp * (-5+float(input))
	I += Ki * (-5+float(input))
	D = Kd * (input - prev_input)
	
	prev_input = input
	
	PID = int(P+I+D)

	if (PID>100): PID = 100
	elif (PID<0): PID = 0

	return PID, I, prev_input

def control_dc_motor(motor_pins, speed, direction):
	if direction == 'forward':
		GPIO.output(motor_pins['IN1'], GPIO.HIGH)
		GPIO.output(motor_pins['IN2'], GPIO.LOW)
		GPIO.output(motor_pins['IN3'], GPIO.HIGH)
		GPIO.output(motor_pins['IN4'], GPIO.LOW)
	elif direction == 'backward':
		GPIO.output(motor_pins['IN1'], GPIO.LOW)
		GPIO.output(motor_pins['IN2'], GPIO.HIGH)
		GPIO.output(motor_pins['IN3'], GPIO.LOW)
		GPIO.output(motor_pins['IN4'], GPIO.HIGH)
	else:
		GPIO.output(motor_pins['IN1'], GPIO.LOW)
		GPIO.output(motor_pins['IN2'], GPIO.LOW)
		GPIO.output(motor_pins['IN3'], GPIO.LOW)
		GPIO.output(motor_pins['IN4'], GPIO.LOW)

	pwmA.start(speed)
	pwmB.start(speed)

# Main loop
MPU_Init()

try:
	while True:
		roll = read_raw_data()
		print(roll)

		vit_mot, I, prev_input = PID(abs(roll), I, prev_input)

		angless = [100, 90, 90, 85]

		# Set servo positions based on angles
		for i in range(4):
			set_servo_angle(servo_pwm[i], angless[i])

		# Control DC motors based on tilt
		if roll < 0:
			control_dc_motor(motor_pins, vit_mot, 'forward')
		elif roll > 0:
			control_dc_motor(motor_pins, vit_mot, 'backward')
		else:
			control_dc_motor(motor_pins, 0, 'stop')

		time.sleep(0.001)

except KeyboardInterrupt:
	pass

finally:
	for pwm in servo_pwm:
		pwm.stop()
	GPIO.cleanup()
