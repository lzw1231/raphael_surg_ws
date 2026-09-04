#include "fd_left_hardware/fd_left_hardware_interface.hpp"
#include <fd_vendor/dhd.hpp>
#include <fd_vendor/drd.hpp>


namespace fd_left_hardware{
    rclcpp::Logger LOGGER = rclcpp::get_logger("FDEffortHardwareInterface");

    FDLeftHardwareInterface::~FDLeftHardwareInterface() {
        on_deactivate(rclcpp_lifecycle::State());
    }

    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_inertia_.resize(15, std::numeric_limits<double>::quiet_NaN());
        hw_button_state_.resize(info_.gpios.size(), std::numeric_limits<double>::quiet_NaN());


        // 验证joints的接口
        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            // PHI has currently exactly 3 states and 1 command interface on each joint
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    LOGGER,
                    "Joint '%s' has %lu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
                RCLCPP_FATAL(
                    LOGGER,
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 3) {
                RCLCPP_FATAL(
                    LOGGER,
                    "Joint '%s' has %ld state interface. 3 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    LOGGER,
                    "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return CallbackReturn::ERROR;
            }
            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    LOGGER,
                    "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return CallbackReturn::ERROR;
            }
            if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
                RCLCPP_FATAL(
                    LOGGER,
                    "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
                return CallbackReturn::ERROR;
            }
        }

        // 验证gpios的接口
        for (const hardware_interface::ComponentInfo& button : info_.gpios) {
            if (button.state_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    LOGGER,
                    "Button '%s' has %lu state interface. 1 expected.", button.name.c_str(),
                    button.state_interfaces.size());
                return CallbackReturn::ERROR;
            }
            if (button.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    LOGGER,
                    "Button '%s' have %s state interface. '%s' expected.", button.name.c_str(),
                    button.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return CallbackReturn::ERROR;
            }
        }

        auto it_interface_serial_number = info_.hardware_parameters.find("interface_serial_number");

        if (it_interface_serial_number != info_.hardware_parameters.end()) {
            interface_SN_ = stoi(it_interface_serial_number->second);
            if (interface_SN_ != SN_LEFT) {
                RCLCPP_FATAL(
                    LOGGER,
                    "Device mismatch: Expected LEFT DEVICE %d, got %d.", SN_LEFT, interface_SN_);
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO(LOGGER, "Using interface serial number: %d", interface_SN_);
        } else {
            interface_SN_ = -1;
        }

        auto it_emulate_button = info_.hardware_parameters.find("emulate_button");
        if (it_emulate_button != info_.hardware_parameters.end()) {
            emulate_button_ = hardware_interface::parse_bool(it_emulate_button->second);
        } else {
            emulate_button_ = false;
        }
        RCLCPP_INFO(LOGGER, "Emulating button: %s", emulate_button_ ? "true" : "false");

        auto it_fd_inertia = info_.hardware_parameters.find("inertia_interface_name");
        if (it_fd_inertia != info_.hardware_parameters.end()) {
            inertia_interface_name_ = it_fd_inertia->second;
        } else {
            inertia_interface_name_ = "fd_inertia";
        }


        return hardware_interface::CallbackReturn::SUCCESS;
    }
}

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB__CLASS_LIST_MACROS_HPP_(fd_left_hardware::FDLeftHardwareInterface
// ,
// hardware_interface::SystemInterface
// )
