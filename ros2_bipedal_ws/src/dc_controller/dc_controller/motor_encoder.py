import RPi.GPIO as GPIO
import time
import threading

class MotorEncoder:
    def __init__(self, pin_a, pin_b, counts_per_rev):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.counts_per_rev = counts_per_rev
        self.counter = 0
        self.rpm = 0
        self.last_time = time.time()
        self.stop_thread = False

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.thread = threading.Thread(target=self._encoder_thread)
        self.thread.start()

    def _encoder_thread(self):
        while not self.stop_thread:
            state_a = GPIO.input(self.pin_a)
            state_b = GPIO.input(self.pin_b)

            if state_a == state_b:
                self.counter += 1
            else:
                self.counter -= 1

            time.sleep(0.001)  # Small delay to prevent high CPU usage

    def calculate_rpm(self):
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        self.last_time = current_time

        # Calculate RPM
        revolutions = self.counter / self.counts_per_rev
        self.counter = 0  # Reset counter after calculating RPM
        self.rpm = (revolutions / elapsed_time) * 60

        return self.rpm

    def cleanup(self):
        self.stop_thread = True
        self.thread.join()
        GPIO.cleanup()


