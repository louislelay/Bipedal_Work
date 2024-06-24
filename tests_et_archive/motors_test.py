import RPi.GPIO as GPIO
import time

# Pin Definitions
enA = 22
in1 = 26
in2 = 6
in3 = 12
in4 = 16
enB = 5

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(enA, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enB, GPIO.OUT)

# Set PWM
pwmA = GPIO.PWM(enA, 1000)
pwmB = GPIO.PWM(enB, 1000)
pwmA.start(0)
pwmB.start(0)



while True:
	pwmA.ChangeDutyCycle(100)
	pwmB.ChangeDutyCycle(100)
	GPIO.output(in1, GPIO.HIGH)
	GPIO.output(in2, GPIO.LOW)
	GPIO.output(in3, GPIO.LOW)
	GPIO.output(in4, GPIO.LOW)
	time.sleep(1)
