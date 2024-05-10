#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "pluginlib/class_loader.hpp"

class TestLoadRaspiServoInterface : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }
};

TEST_F(TestLoadRaspiServoInterface, LoadRaspiServoInterface)
{
    hardware_interface::ResourceManager rm;
    ASSERT_NO_THROW(rm.load_controller("raspi_servo_interface/RaspiServoInterface"));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
