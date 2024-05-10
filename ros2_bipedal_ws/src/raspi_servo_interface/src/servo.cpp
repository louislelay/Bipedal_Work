#include "../include/raspi_servo_interface/servo.hpp"

// Implementation of private member functions
float Servo::degToPWM(float& degAngle) {
    return degAngle * 11.11 + 300;  // In [us]
}

float Servo::radTodeg(float& radAngle) {
    return radAngle * 180 / M_PI;
}

// Implementation of public member functions
Servo::Servo(const int& gpio_pin) {
    GPIO_PIN = gpio_pin;
    // Setting as default pin as OUTPUT due to S90 working principle
    pinMode(GPIO_PIN, OUTPUT);
}

void Servo::goToAngle(float& RadAngle) {
    degAngle = radTodeg(RadAngle);
    pwmOnUs = degToPWM(degAngle);

    digitalWrite(GPIO_PIN, HIGH);
    usleep(pwmOnUs);

    digitalWrite(GPIO_PIN, LOW);
    usleep((pwmPeriodMs*1000 - pwmOnUs));
}

void Servo::continuousSpeed() {
    // Implementation of continuousSpeed function
}