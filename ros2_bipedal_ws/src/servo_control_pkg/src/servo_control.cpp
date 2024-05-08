#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <wiringPi.h>

class ServoControl : public rclcpp::Node
{
public:
    ServoControl() : Node("servo_control"), pin(25)
    {
        wiringPiSetupGpio(); // Use Broadcom pin numbers
        pinMode(pin, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(2000);
        pwmSetClock(192);
        RCLCPP_INFO(this->get_logger(), "Servo control node has been started.");
    }

    void set_servo_angle(int angle)
    {
        int pulse = (angle * 100) / 18 + 150;
        pwmWrite(pin, pulse);
    }

private:
    int pin;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

