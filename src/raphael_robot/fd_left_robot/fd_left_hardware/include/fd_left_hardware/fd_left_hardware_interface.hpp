#pragma once

#include <hardware_interface/system_interface.hpp>
#include "fd_left_hardware/fd_left_hardware_visibility.h"


namespace fd_left_hardware{
    class FDLeftHardwareInterface : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDLeftHardwareInterface);
        virtual ~FDLeftHardwareInterface();

        FD_LEFT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
        FD_LEFT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        FD_LEFT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        FD_LEFT_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        FD_LEFT_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // Store the command for the robot
        std::vector<double> hw_commands_effort_;
        std::vector<double> hw_states_position_;
        std::vector<double> hw_states_velocity_;
        std::vector<double> hw_states_effort_;
        std::vector<double> hw_states_inertia_;
        std::vector<double> hw_button_state_;

        int32_t interface_SN_ = -1;

        bool emulate_button_ = false;

        std::string inertia_interface_name_;

        // fd序列号
        enum SN : int32_t {
            SN_LEFT = 40619,
            SN_RIGHT = 40819
        };
    };
} //namespace fd_left_hardware
