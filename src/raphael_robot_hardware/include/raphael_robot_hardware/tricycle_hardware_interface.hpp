#pragma once

#include <hardware_interface/system_interface.hpp>
#include "SMS_STS.h"

namespace tricycle_hardware{
    class TricycleHardwareInterface : public hardware_interface::SystemInterface {
    public:
        // Lifecycle node override
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        // SystemInterface override
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        std::shared_ptr<SMS_STS> sms_sts_;
        u8 left_motor_id_ = 255;
        u8 right_motor_id_ = 255;
        int baud_ = 0;
        std::string st3215_port_;
    };
} //namespace tricycle_hardware
