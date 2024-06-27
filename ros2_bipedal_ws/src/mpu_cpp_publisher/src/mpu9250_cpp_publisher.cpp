#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "mpu9250_jmdev/registers.hpp"
#include "mpu9250_jmdev/mpu_9250.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class MPU9250Publisher : public rclcpp::Node {
public:
	MPU9250Publisher() : Node("mpu9250_publisher") {
		publisher_ = this->create_publisher<std_msgs::msg::String>("mpu9250_data", 400);
		timer_ = this->create_wall_timer(1s, std::bind(&MPU9250Publisher::timer_callback, this));

		mpu = std::make_unique<MPU9250>(
			AK8963_ADDRESS, 
			MPU9050_ADDRESS_68, 
			-1, 
			1, 
			GFS_1000, 
			AFS_8G, 
			AK8963_BIT_16, 
			AK8963_MODE_C100HZ);

		mpu->configure();  // Apply the settings to the registers.

		pitch = 0.0;
		roll = 0.0;

		last_time = this->now();
	}

private:
	void timer_callback() {
		// Read the accelerometer, gyroscope, and magnetometer uncalibrated values
		accel_data = mpu->readAccelerometerMaster();
		gyro_data = mpu->readGyroscopeMaster();
		mag_data = mpu->readMagnetometerMaster();

		filtered_pitch_roll();

		auto message = std_msgs::msg::String();
		message.data = std::to_string(roll);
		publisher_->publish(message);

		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
	}

	void filtered_pitch_roll() {
		// Calculate roll (rotation around the x-axis)
		roll = std::atan2(accel_data[1], accel_data[2]);
		roll = roll * 180.0 / M_PI;
	}

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	std::unique_ptr<MPU9250> mpu;
	std::vector<float> accel_data;
	std::vector<float> gyro_data;
	std::vector<float> mag_data;

	double pitch;
	double roll;

	rclcpp::Time last_time;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MPU9250Publisher>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
