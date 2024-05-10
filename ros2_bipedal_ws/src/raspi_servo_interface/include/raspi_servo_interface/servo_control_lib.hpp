#ifndef RASPI_SERVO_INTERFACE__SERVO_CONTROL_LIB_HPP_
#define RASPI_SERVO_INTERFACE__SERVO_CONTROL_LIB_HPP_

#include "raspi_servo_interface/visibility_control.h"
#include "raspi_servo_interface/servo.hpp"

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace raspi_servo_interface
{

	class ServoControlLib : public hardware_interface::SystemInterface {

		public:

			RCLCPP_SHARED_PTR_DEFINITIONS(ServoControlLib);

			RASPI_SERVO_INTERFACE_PUBLIC
			hardware_interface::CallbackReturn on_init(
				const hardware_interface::HardwareInfo & info) override;

			RASPI_SERVO_INTERFACE_PUBLIC
			hardware_interface::CallbackReturn on_configure(
				const rclcpp_lifecycle::State & previous_state) override;

			RASPI_SERVO_INTERFACE_PUBLIC
			std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

			RASPI_SERVO_INTERFACE_PUBLIC
			std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

			RASPI_SERVO_INTERFACE_PUBLIC
			hardware_interface::CallbackReturn on_activate(
				const rclcpp_lifecycle::State & previous_state) override;

			RASPI_SERVO_INTERFACE_PUBLIC
			hardware_interface::CallbackReturn on_deactivate(
				const rclcpp_lifecycle::State & previous_state) override;

			RASPI_SERVO_INTERFACE_PUBLIC
			hardware_interface::return_type read(
				const rclcpp::Time & time, const rclcpp::Duration & period) override;

			RASPI_SERVO_INTERFACE_PUBLIC
			hardware_interface::return_type write(
				const rclcpp::Time & time, const rclcpp::Duration & period) override;

		private:
			// Parameters for the S90 servo simulation
			double hw_start_sec_;
			double hw_stop_sec_;
			double hw_slowdown_;

			// Store the command for the simulated robot
			std::vector<double> hw_commands_;
			std::vector<double> hw_states_;
			std::vector<int> GPIO_PINs;

			std::vector<std::unique_ptr<Servo>> servos;
	};

}  // namespace raspi_servo_interface

#endif  // RASPI_SERVO_INTERFACE__SERVO_CONTROL_LIB_HPP_
