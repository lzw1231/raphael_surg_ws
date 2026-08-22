#include "raphael_robot_hardware/tricycle_hardware_interface.hpp"


namespace tricycle_hardware{
    hardware_interface::CallbackReturn TricycleHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        info_ = params.hardware_info;

        left_motor_id_ = 10;
        right_motor_id_ = 20;
        port_ = "/dev/ttyUSB2";

        driver_ = std::make_shared<ZP25SDriver>(port_);
        return hardware_interface::CallbackReturn::SUCCESS;
    } ;

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {};

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {};

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {} ;

    hardware_interface::return_type TricycleHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {} ;

    hardware_interface::return_type TricycleHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {};
} //namespace tricycle_hardware
