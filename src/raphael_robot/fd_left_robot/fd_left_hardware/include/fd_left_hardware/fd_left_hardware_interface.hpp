#pragma once

#include <hardware_interface/system_interface.hpp>
#include "fd_left_hardware/fd_left_hardware_export.h"


namespace fd_left_hardware{
    class FDLeftHardwareInterface : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDLeftHardwareInterface);
        virtual ~FDLeftHardwareInterface();

        FD_LEFT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

        FD_LEFT_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        FD_LEFT_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        FD_LEFT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        FD_LEFT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        FD_LEFT_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        FD_LEFT_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
    };
} //namespace fd_left_hardware
