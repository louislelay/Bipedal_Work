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

def set_motor_a(speed, direction):
    pwmA.ChangeDutyCycle(speed)
    if direction == "forward":
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif direction == "backward":
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)

def set_motor_b(speed, direction):
    pwmB.ChangeDutyCycle(speed)
    if direction == "forward":
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.LOW)
    elif direction == "backward":
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.HIGH)

def stop_motors():
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

try:
    while True:
        set_motor_a(100, "forward")
        set_motor_b(100, "forward")
        print("forward")
        time.sleep(5)

        set_motor_a(75, "backward")
        set_motor_b(75, "backward")
        print("backward")
        time.sleep(5)
        
        set_motor_a(50, "forward")
        set_motor_b(50, "forward")
        print("backward")
        time.sleep(5)
        
        
        set_motor_a(40, "backward")
        set_motor_b(40, "backward")
        print("backward")
        time.sleep(5)
        
        set_motor_a(30, "forward")
        set_motor_b(30, "forward")
        print("backward")
        time.sleep(5)
        
        
        set_motor_a(20, "backward")
        set_motor_b(20, "backward")
        print("backward")
        time.sleep(5)

        set_motor_a(10, "forward")
        set_motor_b(10, "forward")
        print("backward")
        time.sleep(5)

        stop_motors()
        print("stop")
        time.sleep(2)

except KeyboardInterrupt:
    pass

finally:
    stop_motors()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()

