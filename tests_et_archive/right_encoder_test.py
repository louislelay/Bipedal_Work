import RPi.GPIO as GPIO
import time

# Define GPIO pins for encoder A and B outputs
encoder_pin_A = 14
encoder_pin_B = 15

# Set up GPIO
GPIO.setwarnings(False)  # Disable GPIO warnings
GPIO.setmode(GPIO.BCM)
GPIO.setup(encoder_pin_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder_pin_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Global variables to store the state of the encoder
counter = 0
last_state_A = GPIO.input(encoder_pin_A)

def encoder_callback(channel):
    global counter, last_state_A
    state_A = GPIO.input(encoder_pin_A)
    state_B = GPIO.input(encoder_pin_B)
    
    if state_A != last_state_A:  # A has changed
        if state_A == state_B:
            counter += 1
        else:
            counter -= 1
    last_state_A = state_A
    print(f"Counter value: {counter}")

# Add event detection for encoder A
GPIO.add_event_detect(encoder_pin_A, GPIO.BOTH, callback=encoder_callback)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()

