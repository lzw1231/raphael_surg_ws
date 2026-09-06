#include "raphael_hardware/MTM/fd_left/fd_left_hardware.hpp"

namespace fd_left_hardware{
    rclcpp::Logger LOGGER = rclcpp::get_logger("FDLeftHardwareInterface");

    FDLeftHardwareInterface::~FDLeftHardwareInterface() {
        // If controller manager is shutdown via Ctrl + C, the on_deactivate methods won't be called.
        // We need to call them here to ensure that the device is stopped and disconnected.
        FDLeftHardwareInterface::on_deactivate(rclcpp_lifecycle::State());
    }

    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        
        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_inertia_.resize(15, std::numeric_limits<double>::quiet_NaN());
        hw_button_state_.resize(info_.gpios.size(), std::numeric_limits<double>::quiet_NaN());

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state; // hush "-Wunused-parameter" warning
        RCLCPP_INFO(LOGGER, "Activating ...please wait...");
        // Add activation logic here
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state; // hush "-Wunused-parameter" warning
        RCLCPP_INFO(LOGGER, "Deactivating ...please wait...");
        // Add deactivation logic here
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type FDLeftHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&) {
        // Add read logic here
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type FDLeftHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
        // Add write logic here
        return hardware_interface::return_type::OK;
    }
} // namespace fd_left_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fd_left_hardware::FDLeftHardwareInterface, hardware_interface::SystemInterface)
