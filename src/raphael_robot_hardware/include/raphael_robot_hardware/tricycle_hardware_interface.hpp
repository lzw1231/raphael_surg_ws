#pragma once

#include <hardware_interface/system_interface.hpp>
#include "raphael_robot_hardware/zp25s_driver.hpp"

namespace tricycle_hardware{
    class TricycleHardwareInterface : public hardware_interface::SystemInterface {
    public:
        //Lifecycle node override
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        //SystemInterface override
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        std::shared_ptr<ZP25SDriver> driver_;
        int left_motor_id_;
        int right_motor_id_;
        std::string port_;
    }; // class TricycleHardwareInterface
} //namespace tricycle_hardware
