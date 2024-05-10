#ifndef SERVO_HPP
#define SERVO_HPP

#include <wiringPi.h>
#include <unistd.h>
#include <cstdio>
#include <math.h>


class Servo {
    private:
        int GPIO_PIN;
        float pwmOnUs{0.0}; // ms on which the squarewave will stay up in ms
        float pwmPeriodMs{20.0}; // Pwm period in ms
        float degAngle;

        float degToPWM(float &degAngle);

        float radTodeg(float &radAngle);

    public:
        Servo(const int &gpio_pin);

        void goToAngle(float &RadAngle);

        void continuousSpeed();

};

#endif // SERVO_HPP