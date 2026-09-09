#include "raphael_hardware/common/fd_utils.hpp"
#include <fd_vendor/fd_sdk.hpp>


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

    FDConnectResult connect_to_device(rclcpp::Logger logger, const FDConnectParams& params) {
        FDConnectResult res{};
        int major, minor, release, revision;
        dhdGetSDKVersion(&major, &minor, &release, &revision);
        RCLCPP_INFO(logger,
                    "DHD SDK 版本: %d.%d (release %d / revision %d)", major, minor, release, revision);

        bool dhd_open_success = false;
        int interface_id = -1;
        if (params.interface_sn >= 0) {
            RCLCPP_INFO(logger, "尝试通过序列号 %d 打开设备...", params.interface_sn);
            interface_id = static_cast<int>(dhdOpenSerial(params.interface_sn));
            dhd_open_success = (interface_id >= 0);
        }

        if (!dhd_open_success) {
            RCLCPP_ERROR(logger, "打开设备失败");
            res.success = false;
            return res;
        }

        RCLCPP_INFO(logger, "设备名称: %s", dhdGetSystemName(interface_id));
        uint16_t serialNumber = 0;
        if (dhdGetSerialNumber(&serialNumber, interface_id) < 0) {
            RCLCPP_WARN(logger, "无法获取序列号: %s", dhdErrorGetLastStr());
        } else {
            RCLCPP_INFO(logger, "设备序列号: %d", serialNumber);
        }
        RCLCPP_INFO(logger, "内部接口 ID: %d", interface_id);

        bool has_wrist = (dhdHasWrist(interface_id) != 0);
        if (has_wrist) {
            RCLCPP_INFO(logger, "检测到腕部自由度");
        } else {
            RCLCPP_INFO(logger, "未检测到腕部自由度");
        }

        double current_effector_mass = 0.0;
        if (dhdGetEffectorMass(&current_effector_mass, interface_id) == DHD_NO_ERROR) {
            RCLCPP_INFO(logger, "当前末端质量: %.2f g", current_effector_mass * 1000.0);
        } else {
            RCLCPP_WARN(logger, "无法获取末端质量");
        }

        if (dhdSetMaxForce(params.max_force, interface_id) < DHD_NO_ERROR) {
            RCLCPP_ERROR(logger, "设置最大力失败");
            (void)disconnect_from_device(logger, interface_id);
            res.success = false;
            return res;
        }

        dhdSetBrakes(DHD_OFF, interface_id);

        if (dhdEnableForce(DHD_ON, interface_id) < DHD_NO_ERROR) {
            RCLCPP_ERROR(logger, "启用力反馈失败");
            (void)disconnect_from_device(logger, interface_id);
            res.success = false;
            return res;
        }

        if (params.effector_mass > 0.0) {
            RCLCPP_INFO(logger,
                        "更新末端质量: %.2f g -> %.2f g",
                        current_effector_mass * 1000.0,
                        params.effector_mass * 1000.0);
            if (dhdSetEffectorMass(params.effector_mass, interface_id) < DHD_NO_ERROR) {
                RCLCPP_ERROR(logger, "设置末端质量失败");
                (void)disconnect_from_device(logger, interface_id);
                res.success = false;
                return res;
            }
        }

        if (dhdSetGravityCompensation(DHD_ON, interface_id) < DHD_NO_ERROR) {
            RCLCPP_ERROR(logger, "开启重力补偿失败");
            (void)disconnect_from_device(logger, interface_id);
            res.success = false;
            return res;
        }
        RCLCPP_INFO(logger, "重力补偿已开启");
        RCLCPP_INFO(logger, "设备基本配置完成");

        bool has_gripper = (dhdHasGripper(interface_id) != 0);
        if (params.emulate_button && !has_gripper) {
            RCLCPP_ERROR(logger, "启用按键模拟但设备无夹爪");
        } else if (params.emulate_button && has_gripper) {
            RCLCPP_INFO(logger, "设备带有夹爪，启用按键模拟");
            if (dhdEmulateButton(DHD_ON, interface_id) < DHD_NO_ERROR) {
                RCLCPP_ERROR(logger, "启用按键模拟失败");
                (void)disconnect_from_device(logger, interface_id);
                res.success = false;
                return res;
            }
            RCLCPP_INFO(logger, "按键模拟功能已激活");
        }

        if (dhdSetForceAndTorqueAndGripperForce(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            interface_id) < DHD_NO_ERROR) {
            RCLCPP_ERROR(logger, "初始化力输出失败");
            (void)disconnect_from_device(logger, interface_id);
            res.success = false;
            return res;
        }

        // 根据硬件能力自动设置忽略姿态
        bool ignore_orient = !has_wrist;
        dhdSleep(0.1);

        res.interface_id = interface_id;
        res.ignore_orientation = ignore_orient;
        res.success = true;
        return res;
    }

    bool disconnect_from_device(rclcpp::Logger logger, int& interface_id) {
        if (interface_id < 0) {
            RCLCPP_WARN(logger, "设备ID已经无效，无需断开");
            return true;
        }

        int hasStopped = -1;
        while (hasStopped < 0) {
            RCLCPP_INFO(logger, "正在停止 DHD 设备...");
            hasStopped = dhdStop(interface_id);
            dhdSleep(0.1);
        }

        int connectionIsClosed = dhdClose(interface_id);
        if (connectionIsClosed >= 0) {
            RCLCPP_INFO(logger, "DHD 设备已关闭");
            interface_id = -1;
            return true;
        } else {
            RCLCPP_ERROR(logger, "DHD 设备关闭失败");
            return false;
        }
    }
} // namespace fd_utils
