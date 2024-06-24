import RPi.GPIO as GPIO
import time

# Define GPIO pins for encoder A and B outputs
encoder_pin_A = 14
encoder_pin_B = 15

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(encoder_pin_A, GPIO.IN)
GPIO.setup(encoder_pin_B, GPIO.IN)

while True:
	print(GPIO.input(encoder_pin_A))
	print(GPIO.input(encoder_pin_A))
	time.sleep(0.1)