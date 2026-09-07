#include "raphael_hardware/MTM/fd_left/fd_left_hardware.hpp"
#include <fd_vendor/dhd.hpp>
#include <fd_vendor/drd.hpp>

namespace fd_left_hardware{
    unsigned int flattened_index_from_triangular_index(unsigned int idx_row, unsigned int idx_col, unsigned int dim = 6) {
        unsigned int i = idx_row;
        unsigned int j = idx_col;
        if (idx_col < idx_row) {
            i = idx_col;
            j = idx_row;
        }
        return i * (2 * dim - i - 1) / 2 + j;
    }

    rclcpp::Logger LOGGER = rclcpp::get_logger("FDLeftHardwareInterface");

    FDLeftHardwareInterface::~FDLeftHardwareInterface() {
        (void)disconnectFromDevice();
    }

    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_inertia_.resize(21, std::numeric_limits<double>::quiet_NaN());
        hw_button_state_.resize(info_.gpios.size(), std::numeric_limits<double>::quiet_NaN());

        // 遍历所有关节，校验ros2_control配置的state/command接口是否匹配硬件要求
        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            // command接口数量校验，期望1个
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    LOGGER,
                    "关节 '%s' 检测到 %lu 个command接口，期望为 1 个", joint.name.c_str(),
                    joint.command_interfaces.size());
                return CallbackReturn::ERROR;
            }
            // command接口类型校验，期望effort
            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
                RCLCPP_FATAL(
                    LOGGER,
                    "关节 '%s' command接口实际类型为[%s]，期望类型为[%s]", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
                return CallbackReturn::ERROR;
            }
            // state接口数量校验，期望3个
            if (joint.state_interfaces.size() != 3) {
                RCLCPP_FATAL(
                    LOGGER,
                    "关节 '%s' 检测到 %ld 个state接口，期望为 3 个", joint.name.c_str(),
                    joint.state_interfaces.size());
                return CallbackReturn::ERROR;
            }
            // state[0]接口类型校验，期望position
            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    LOGGER,
                    "关节 '%s' state[0]接口实际类型为[%s]，期望类型为[%s]", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return CallbackReturn::ERROR;
            }
            // state[1]接口类型校验，期望velocity
            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    LOGGER,
                    "关节 '%s' state[1]接口实际类型为[%s]，期望类型为[%s]", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return CallbackReturn::ERROR;
            }
            // state[2]接口类型校验，期望effort
            if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
                RCLCPP_FATAL(
                    LOGGER,
                    "关节 '%s' state[2]接口实际类型为[%s]，期望类型为[%s]", joint.name.c_str(),
                    joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
                return CallbackReturn::ERROR;
            }
        }

        // 遍历GPIO按键，校验state接口配置
        for (const hardware_interface::ComponentInfo& button : info_.gpios) {
            // state接口数量校验，期望1个
            if (button.state_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    LOGGER,
                    "按键 '%s' 检测到 %lu 个state接口，期望为 1 个", button.name.c_str(),
                    button.state_interfaces.size());
                return CallbackReturn::ERROR;
            }
            // state接口类型校验，期望position
            if (button.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    LOGGER,
                    "按键 '%s' state接口实际类型为[%s]，期望类型为[%s]", button.name.c_str(),
                    button.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return CallbackReturn::ERROR;
            }
        }

        // 读取硬件ros2_control配置参数
        auto it_interface_serial_number = info_.hardware_parameters.find("interface_serial_number");
        if (it_interface_serial_number != info_.hardware_parameters.end()) {
            interface_SN_ = stoi(it_interface_serial_number->second);
            RCLCPP_INFO(LOGGER, "使用设备 serial_number: %d", interface_SN_);
        } else {
            interface_SN_ = -1;
        }

        auto it_emulate_button = info_.hardware_parameters.find("emulate_button");
        if (it_emulate_button != info_.hardware_parameters.end()) {
            emulate_button_ = hardware_interface::parse_bool(it_emulate_button->second);
        } else {
            emulate_button_ = false;
        }
        RCLCPP_INFO(LOGGER, "按键仿真开关: %s", emulate_button_ ? "true" : "false");

        auto it_fd_inertia = info_.hardware_parameters.find("inertia_interface_name");
        if (it_fd_inertia != info_.hardware_parameters.end()) {
            inertia_interface_name_ = it_fd_inertia->second;
        } else {
            inertia_interface_name_ = "fd_inertia";
        }

        auto it_interface_mass = info_.hardware_parameters.find("effector_mass");
        if (it_interface_mass != info_.hardware_parameters.end()) {
            effector_mass_ = hardware_interface::stod(it_interface_mass->second);
            RCLCPP_INFO(LOGGER, "末端执行器质量参数: %lf Kg", effector_mass_);
        } else {
            effector_mass_ = -1.0;
        }

        auto it_ignore_orientation = info_.hardware_parameters.find("ignore_orientation_readings");
        if (it_ignore_orientation != info_.hardware_parameters.end()) {
            ignore_orientation_ = hardware_interface::parse_bool(it_ignore_orientation->second);
        } else {
            ignore_orientation_ = false;
        }
        RCLCPP_INFO(LOGGER, "忽略姿态读数: %s", ignore_orientation_ ? "true" : "false");

        // 按键仿真模式下，特殊处理离合器关节力矩指令，防止输出NaN
        if (emulate_button_ && (info_.joints.size() == 4 || info_.joints.size() > 6)) {
            // 防止离合器指令出现NaN
            hw_commands_effort_[info_.joints.size() - 1] = 0.0;
        }

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;
        RCLCPP_INFO(LOGGER, "正在启动力反馈设备，请稍候...");

        if (connectToDevice()) {
            RCLCPP_INFO(LOGGER, "力反馈设备启动成功，硬件链路已建立！");
            return CallbackReturn::SUCCESS;
        } else {
            RCLCPP_ERROR(LOGGER, "力反馈设备启动失败，硬件连接异常！");
            return CallbackReturn::ERROR;
        }
    }

    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;
        RCLCPP_INFO(LOGGER, "正在停止力反馈设备，请稍候...");

        if (disconnectFromDevice()) {
            RCLCPP_INFO(LOGGER, "设备已成功停止，硬件链路断开！");
            return CallbackReturn::SUCCESS;
        } else {
            RCLCPP_ERROR(LOGGER, "设备停止失败，硬件链路断开异常！");
            return CallbackReturn::ERROR;
        }
    }

    hardware_interface::return_type FDLeftHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;
        (void)period;

        int flag = 0; // 接口返回值累加，小于0代表存在底层API调用失败

        // ---------- 读取笛卡尔位置与姿态 ----------
        flag += dhdGetPosition(&hw_states_position_[0], &hw_states_position_[1], &hw_states_position_[2], interface_ID_);

        if (!ignore_orientation_ && hw_states_position_.size() > 3) {
            // 硬件支持姿态，读取旋转角(rad)
            flag += dhdGetOrientationRad(&hw_states_position_[3], &hw_states_position_[4], &hw_states_position_[5], interface_ID_);
        } else if (ignore_orientation_ && hw_states_position_.size() > 3) {
            // 姿态忽略标志置位，姿态分量强制清零
            hw_states_position_[3] = 0.0;
            hw_states_position_[4] = 0.0;
            hw_states_position_[5] = 0.0;
        }

        // ---------- 读取夹持器角度 ----------
        if (dhdHasGripper(interface_ID_)) {
            if (hw_states_position_.size() == 4) {
                flag += dhdGetGripperAngleRad(&hw_states_position_[3], interface_ID_);
            } else if (hw_states_position_.size() > 6) {
                flag += dhdGetGripperAngleRad(&hw_states_position_[6], interface_ID_);
            }
        }

        // ---------- 读取线速度、角速度 ----------
        flag += dhdGetLinearVelocity(&hw_states_velocity_[0], &hw_states_velocity_[1], &hw_states_velocity_[2], interface_ID_);

        if (!ignore_orientation_ && hw_states_velocity_.size() > 3) {
            // 读取腕部角速度
            flag += dhdGetAngularVelocityRad(&hw_states_velocity_[3], &hw_states_velocity_[4], &hw_states_velocity_[5], interface_ID_);
        } else if (ignore_orientation_ && hw_states_velocity_.size() > 3) {
            // 忽略姿态，角速度分量清零
            hw_states_velocity_[3] = 0.0;
            hw_states_velocity_[4] = 0.0;
            hw_states_velocity_[5] = 0.0;
        }

        // ---------- 读取夹持器角速度 ----------
        if (dhdHasGripper(interface_ID_)) {
            if (hw_states_velocity_.size() == 4) {
                flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[3], interface_ID_);
            } else if (hw_states_velocity_.size() > 6) {
                flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[6], interface_ID_);
            }
        }

        // ---------- 读取设备反馈力、力矩、夹持力 ----------
        double torque[3];
        double gripper_force;
        flag += dhdGetForceAndTorqueAndGripperForce(
            &hw_states_effort_[0], &hw_states_effort_[1], &hw_states_effort_[2],
            &torque[0], &torque[1], &torque[2],
            &gripper_force,
            interface_ID_
        );

        if (!ignore_orientation_ && hw_states_effort_.size() > 3) {
            // 拷贝腕部力矩到状态数组
            hw_states_effort_[3] = torque[0];
            hw_states_effort_[4] = torque[1];
            hw_states_effort_[5] = torque[2];
        } else if (ignore_orientation_ && hw_states_effort_.size() > 3) {
            // 忽略姿态，力矩分量清零
            hw_states_effort_[3] = 0.0;
            hw_states_effort_[4] = 0.0;
            hw_states_effort_[5] = 0.0;
        }

        // ---------- 拷贝夹持器力 ----------
        if (dhdHasGripper(interface_ID_)) {
            if (hw_states_effort_.size() == 4) {
                hw_states_effort_[3] = gripper_force;
            } else if (hw_states_effort_.size() > 6) {
                hw_states_effort_[6] = gripper_force;
            }
        }

        // ---------- 读取设备动力学：笛卡尔6×6惯性矩阵(专家模式接口) ----------
        double inertia_array[6][6];
        double joint_position[DHD_MAX_DOF]; // DHD_MAX_DOF=8，SDK预留最大8路DOF；Omega7实际只有7个电机，joint_position[7]未使用

        // 开启专家模式：解锁底层动力学API，dhdGetJointAngles仅专家模式可用
        flag += dhdEnableExpertMode();
        // 读取硬件全部电机机械关节角，注意：不是笛卡尔末端位姿，仅用于SDK动力学运算
        flag += dhdGetJointAngles(joint_position, interface_ID_);
        // 根据电机关节角，计算末端笛卡尔空间6×6惯性矩阵(不含夹持器gripper轴)
        flag += dhdJointAnglesToInertiaMatrix(joint_position, inertia_array, interface_ID_);
        // 用完立即关闭专家模式，切回普通安全模式，禁止长期驻留专家模式
        flag += dhdDisableExpertMode();


        // 将惯性矩阵上三角展开，映射到一维惯性状态接口
        for (uint row = 0; row < 6; row++) {
            for (uint col = row; col < 6; col++) {
                hw_states_inertia_[flattened_index_from_triangular_index(row, col)] = inertia_array[row][col];
            }
        }

        // ---------- 读取模拟按键状态  ----------
        int button_status = dhdGetButton(0, interface_ID_);
        if (button_status == 1) {
            hw_button_state_[0] = 1.0;
        } else if (button_status == 0) {
            hw_button_state_[0] = 0.0;
        } else {
            RCLCPP_ERROR(LOGGER, "模拟按键读取异常！");
            flag += -1;
        }

        // ---------- 结果判断 ----------
        if (flag >= 0) {
            return hardware_interface::return_type::OK;
        } else {
            RCLCPP_ERROR(LOGGER, "硬件状态读取失败！底层DHD接口返回错误");
            return hardware_interface::return_type::ERROR;
        }
    }


    hardware_interface::return_type FDLeftHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;
        (void)period;

        // 指令非法值标记，检测力指令是否存在NaN无效数据
        bool isNan = false;

        // 遍历所有力控指令，检测NaN异常（浮点数NaN不等于自身）
        for (auto& command : hw_commands_effort_) {
            if (command != command) {
                isNan = true;
            }
        }

        // 无NaN异常时，正常执硬件力指令输出
        if (!isNan) {
            // 设备带夹持器且硬件自由度大于6：输出完整6维力力矩+夹持器力
            if (dhdHasGripper(interface_ID_) && hw_states_effort_.size() > 6) {
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5],
                    hw_commands_effort_[6], interface_ID_);
            }
            // 设备带腕部、硬件自由度为4：仅输出平动力+夹持力，屏蔽旋转力矩
            else if (dhdHasWrist(interface_ID_) && hw_states_effort_.size() == 4) {
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    0.0, 0.0, 0.0, hw_commands_effort_[3], interface_ID_);
            }
            // 设备带腕部、无夹持离合关节：输出完整6维力力矩，屏蔽夹持器力
            else if (dhdHasWrist(interface_ID_) && hw_states_effort_.size() > 3) {
                // 无夹持离合关节，禁用夹持力反馈，仅开启6DOF力力矩反馈
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5],
                    0, interface_ID_);
            }
            // 基础平移构型设备：仅输出三维平动力，屏蔽所有力矩与夹持力
            else {
                // 仅平移维度作动，关闭旋转与夹持力反馈
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    0, 0, 0, 0, interface_ID_);
            }
        }
        // 检测到NaN非法指令，执行硬件安全保护，清零所有力反馈输出
        else {
            dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, interface_ID_);
        }

        // 硬件写入执行完成，返回正常状态
        return hardware_interface::return_type::OK;
    }


    bool FDLeftHardwareInterface::connectToDevice() {
        int major, minor, release, revision;
        // 获取底层DHD SDK版本信息
        dhdGetSDKVersion(&major, &minor, &release, &revision);
        RCLCPP_INFO(
            LOGGER,
            "DHD SDK 版本: %d.%d (release %d / revision %d)",
            major, minor, release, revision);

        bool dhd_open_success = false;
        // 根据配置参数选择设备打开方式：序列号指定 / ID指定 / 默认设备
        if (interface_SN_ >= 0) {
            // 通过序列号打开指定设备
            RCLCPP_INFO(LOGGER, "正在通过序列号 %d 连接力反馈设备，请稍候...", interface_SN_);
            interface_ID_ = static_cast<char>(dhdOpenSerial(interface_SN_));
            dhd_open_success = (interface_ID_ >= 0);
        }

        if (dhd_open_success) {
            RCLCPP_INFO(LOGGER, "检测到设备: %s", dhdGetSystemName(interface_ID_));

            // 读取设备序列号
            uint16_t serialNumber = 0;
            if (dhdGetSerialNumber(&serialNumber, interface_ID_) < 0) {
                RCLCPP_WARN(LOGGER, "读取设备序列号失败: %s", dhdErrorGetLastStr());
            } else {
                RCLCPP_INFO(LOGGER, "设备序列号: %d", serialNumber);
            }
            RCLCPP_INFO(LOGGER, "设备会话ID: %d", interface_ID_);

            // 判断设备是否支持腕部旋转自由度
            if (dhdHasWrist(interface_ID_)) {
                RCLCPP_INFO(LOGGER, "设备支持腕部旋转");
            } else {
                RCLCPP_INFO(LOGGER, "设备不支持腕部旋转");
            }

            // 读取设备出厂末端执行器质量
            double current_effector_mass = 0.0;
            if (dhdGetEffectorMass(&current_effector_mass, interface_ID_) == DHD_NO_ERROR) {
                RCLCPP_INFO(LOGGER, "出厂末端质量: %.2f g", current_effector_mass * 1000.0);
            } else {
                RCLCPP_WARN(LOGGER, "读取出厂末端质量失败");
            }

            // 设置最大输出力阈值
            double forceMax = 12; // 单位:N
            if (dhdSetMaxForce(forceMax, interface_ID_) < DHD_NO_ERROR) {
                RCLCPP_ERROR(LOGGER, "设置最大输出力失败，执行断开设备");
                disconnectFromDevice();
            }

            // 解除机械制动，使能力矩输出
            dhdSetBrakes(DHD_OFF, interface_ID_);
            if (dhdEnableForce(DHD_ON, interface_ID_) < DHD_NO_ERROR) {
                RCLCPP_ERROR(LOGGER, "使能力反馈控制失败，执行断开设备");
                disconnectFromDevice();
                return false;
            }

            // 用户自定义末端执行器质量配置
            if (effector_mass_ > 0.0) {
                RCLCPP_INFO(
                    LOGGER,
                    "末端质量由 %.2f g 修改为 %.2f g",
                    current_effector_mass * 1000.0,
                    effector_mass_ * 1000.0);
                if (dhdSetEffectorMass(effector_mass_, interface_ID_) < DHD_NO_ERROR) {
                    RCLCPP_ERROR(LOGGER, "设置自定义末端质量失败，执行断开设备");
                    disconnectFromDevice();
                    return false;
                }
            }

            // 开启重力补偿
            if (dhdSetGravityCompensation(DHD_ON, interface_ID_) < DHD_NO_ERROR) {
                RCLCPP_ERROR(LOGGER, "开启重力补偿失败，执行断开设备");
                disconnectFromDevice();
                return false;
            } else {
                RCLCPP_INFO(LOGGER, "重力补偿已开启");
            }

            RCLCPP_INFO(LOGGER, "力反馈设备通信链路建立完成");

            // 夹持器关节模拟物理按键功能
            if (emulate_button_ && !dhdHasGripper(interface_ID_)) {
                RCLCPP_ERROR(LOGGER, "未检测到夹持器，无法启用按键模拟功能");
            } else if (emulate_button_ && dhdHasGripper(interface_ID_)) {
                RCLCPP_INFO(LOGGER, "启用夹持器关节按键模拟");
                if (dhdEmulateButton(DHD_ON, interface_ID_) < DHD_NO_ERROR) {
                    RCLCPP_ERROR(LOGGER, "按键模拟功能开启失败，执行断开设备");
                    disconnectFromDevice();
                    return false;
                }
                RCLCPP_INFO(LOGGER, "按键模拟配置完成，由夹持器关节映射按键信号");
            }

            // 初始化输出力矩、扭矩、夹持力全部置零
            if (dhdSetForceAndTorqueAndGripperForce(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                interface_ID_) < DHD_NO_ERROR) {
                RCLCPP_ERROR(LOGGER, "力矩输出初始化置零失败，执行断开设备");
                disconnectFromDevice();
                return false;
            }

            // 根据设备硬件能力标记是否忽略姿态输出
            ignore_orientation_ |= !dhdHasWrist(interface_ID_);
            if (ignore_orientation_) {
                RCLCPP_INFO(LOGGER, "设备无腕部自由度，姿态数据将被忽略");
            }

            // 等待硬件状态稳定
            dhdSleep(0.1);
            isConnected_ = true;
            return isConnected_;
        } else {
            RCLCPP_ERROR(LOGGER, "力反馈设备打开失败");
            isConnected_ = false;
            return isConnected_;
        }
    }

    bool FDLeftHardwareInterface::disconnectFromDevice() {
        // 执行设备安全停机：关闭力反馈输出，设备进入制动模式
        int hasStopped = -1;
        // 循环等待设备停机完成，dhdStop返回非负数代表停机成功
        while (hasStopped < 0) {
            RCLCPP_INFO(LOGGER, "DHD：正在执行设备停机，请稍候...");
            hasStopped = dhdStop(interface_ID_);
            dhdSleep(0.1); // 等待硬件状态响应，阻塞延时，仅允许在非实时上下文调用
        }

        // 关闭USB设备会话，释放底层设备资源
        int connectionIsClosed = dhdClose(interface_ID_);
        if (connectionIsClosed >= 0) {
            RCLCPP_INFO(LOGGER, "DHD：设备已断开连接");
            interface_ID_ = -1; // 重置会话ID，标记设备无效
            isConnected_ = false;
            return true;
        } else {
            RCLCPP_ERROR(LOGGER, "DHD：设备断开失败！底层句柄关闭异常");
            return false;
        }
    }
} // namespace fd_left_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fd_left_hardware::FDLeftHardwareInterface, hardware_interface::SystemInterface)
