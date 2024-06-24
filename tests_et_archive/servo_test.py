import RPi.GPIO as GPIO
import time

# Pin Definitions
servo_pin = 18

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# Set PWM
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz PWM frequency
pwm.start(0)

def set_servo_angle(angle):
    duty_cycle = (0.05 * 50) + (0.19 * 50 * angle / 180)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Allow servo to reach the position

try:
    while True:
        # Rotate servo to 0 degrees
        set_servo_angle(0)
        time.sleep(2)

        # Rotate servo to 90 degrees
        set_servo_angle(90)
        time.sleep(2)

        # Rotate servo to 180 degrees
        set_servo_angle(180)
        time.sleep(2)

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()

