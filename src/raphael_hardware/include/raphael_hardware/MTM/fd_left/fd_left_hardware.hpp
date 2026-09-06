#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/macros.hpp>
#include "raphael_hardware/visibility_control.hpp"


namespace fd_left_hardware{
    class FDLeftHardwareInterface : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDLeftHardwareInterface);
        ~FDLeftHardwareInterface() override;

        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;

        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

    private:
        hardware_interface::HardwareInfo hw_info_;

        std::vector<double> hw_commands_effort_;
        std::vector<double> hw_states_position_;
        std::vector<double> hw_states_velocity_;
        std::vector<double> hw_states_effort_;
        std::vector<double> hw_states_inertia_; // upper-triangular matrix
        std::vector<double> hw_button_state_;
    }; // class FDLeftHardwareInterface
} // namespace fd_left_hardware
