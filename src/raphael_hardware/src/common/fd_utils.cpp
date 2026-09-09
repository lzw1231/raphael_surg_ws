#include "raphael_hardware/common/fd_utils.hpp"


namespace fd_utils{
    bool validate_joint_interfaces(
        const hardware_interface::ComponentInfo& joint, rclcpp::Logger& logger) {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                logger,
                "Joint '%s' 需要 1 个指令接口，当前 %zu 个",
                joint.name.c_str(), joint.command_interfaces.size());
            return false;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(
                logger,
                "Joint '%s' 指令接口类型错误: 期望 '%s', 实际 '%s'",
                joint.name.c_str(),
                hardware_interface::HW_IF_EFFORT,
                joint.command_interfaces[0].name.c_str());
            return false;
        }
        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(
                logger,
                "Joint '%s' 需要 3 个状态接口，当前 %zu 个",
                joint.name.c_str(), joint.state_interfaces.size());
            return false;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                logger,
                "Joint '%s' 状态接口[0]类型错误: 期望 '%s', 实际 '%s'",
                joint.name.c_str(),
                hardware_interface::HW_IF_POSITION,
                joint.state_interfaces[0].name.c_str());
            return false;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                logger,
                "Joint '%s' 状态接口[1]类型错误: 期望 '%s', 实际 '%s'",
                joint.name.c_str(),
                hardware_interface::HW_IF_VELOCITY,
                joint.state_interfaces[1].name.c_str());
            return false;
        }
        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(
                logger,
                "Joint '%s' 状态接口[2]类型错误: 期望 '%s', 实际 '%s'",
                joint.name.c_str(),
                hardware_interface::HW_IF_EFFORT,
                joint.state_interfaces[2].name.c_str());
            return false;
        }
        return true;
    }

    bool validate_gpio_interfaces(const hardware_interface::ComponentInfo& button, rclcpp::Logger& logger) {
        if (button.state_interfaces.size() != 1) {
            RCLCPP_FATAL(
                logger,
                "GPIO '%s' 需要 1 个 state 接口，当前 %lu",
                button.name.c_str(),
                button.state_interfaces.size());
            return false;
        }
        if (button.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                logger,
                "GPIO '%s' state 接口类型错误 期望 [%s] 实际 [%s]",
                button.name.c_str(),
                hardware_interface::HW_IF_POSITION,
                button.state_interfaces[0].name.c_str());
            return false;
        }
        return true;
    }

    bool validate_hardware_interfaces(const hardware_interface::HardwareInfo& info, rclcpp::Logger& logger) {
        for (const hardware_interface::ComponentInfo& joint : info.joints) {
            if (!validate_joint_interfaces(joint, logger)) {
                return false;
            }
        }

        for (const hardware_interface::ComponentInfo& button : info.gpios) {
            if (!validate_gpio_interfaces(button, logger)) {
                return false;
            }
        }
        return true;
    }

    FDHardwareParams load_hardware_parameters(const hardware_interface::HardwareInfo& info, rclcpp::Logger& logger) {
        FDHardwareParams params;

        auto it_interface_sn = info.hardware_parameters.find("interface_sn");
        if (it_interface_sn != info.hardware_parameters.end()) {
            params.interface_sn = std::stoi(it_interface_sn->second);
            RCLCPP_INFO(logger, "接口 sn: %d", params.interface_sn);
        } else {
            params.interface_sn = -1;
        }

        auto it_emulate_button = info.hardware_parameters.find("emulate_button");
        if (it_emulate_button != info.hardware_parameters.end()) {
            params.emulate_button = hardware_interface::parse_bool(it_emulate_button->second);
        } else {
            params.emulate_button = false;
        }
        RCLCPP_INFO(logger, "模拟按键: %s", params.emulate_button ? "true" : "false");

        auto it_fd_inertia = info.hardware_parameters.find("inertia_interface_name");
        if (it_fd_inertia != info.hardware_parameters.end()) {
            params.inertia_interface_name = it_fd_inertia->second;
        } else {
            params.inertia_interface_name = "fd_inertia";
        }

        auto it_interface_mass = info.hardware_parameters.find("effector_mass");
        if (it_interface_mass != info.hardware_parameters.end()) {
            params.effector_mass = hardware_interface::stod(it_interface_mass->second);
            RCLCPP_INFO(logger, "末端质量: %lf Kg", params.effector_mass);
        } else {
            params.effector_mass = -1.0;
        }

        auto it_ignore_orientation = info.hardware_parameters.find("ignore_orientation_readings");
        if (it_ignore_orientation != info.hardware_parameters.end()) {
            params.ignore_orientation = hardware_interface::parse_bool(it_ignore_orientation->second);
        } else {
            params.ignore_orientation = false;
        }
        RCLCPP_INFO(logger, "忽略姿态读取: %s", params.ignore_orientation ? "true" : "false");

        return params;
    }
} // namespace fd_utils
