#include "raphael_hardware/mtm/fd_left/fd_left_hardware.hpp"
#include "raphael_hardware/common/math_utils.hpp"
#include "raphael_hardware/common/fd_utils.hpp"
#include <fd_vendor/fd_sdk.hpp>


namespace fd_left_hardware{
    rclcpp::Logger LOGGER = rclcpp::get_logger("FDLeftHardwareInterface");

    FDLeftHardwareInterface::~FDLeftHardwareInterface() {
        int id = static_cast<int>(interface_ID_);
        fd_utils::disconnect_from_device(LOGGER, id);
        interface_ID_ = static_cast<char>(id);
        isConnected_ = false;
    }

    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        if (auto ret = hardware_interface::SystemInterface::on_init(params);
            ret != hardware_interface::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(LOGGER, "SystemInterface 初始化失败: %d 检查 URDF 配置", static_cast<int>(ret));
            return ret;
        }
        info_ = get_hardware_info();

        // 调用工具函数校验接口
        if (!fd_utils::validate_hardware_interfaces(info_, LOGGER)) {
            return CallbackReturn::ERROR;
        }

        // 读取参数到结构体，再赋值给类成员
        auto fd_params = fd_utils::load_hardware_parameters(info_, LOGGER);
        interface_SN_ = fd_params.interface_sn;
        emulate_button_ = fd_params.emulate_button;
        inertia_interface_name_ = fd_params.inertia_interface_name;
        effector_mass_ = fd_params.effector_mass;
        ignore_orientation_ = fd_params.ignore_orientation;

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // 调用基类 configure
        if (auto ret = hardware_interface::SystemInterface::on_configure(previous_state);
            ret != hardware_interface::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(LOGGER, "SystemInterface::on_configure 失败");
            return ret;
        }

        // ========== 1. 初始化状态/命令缓冲区 ==========
        hw_math::resize_and_fill(hw_states_position_, info_.joints.size(), BUF_INIT_NAN);
        hw_math::resize_and_fill(hw_states_velocity_, info_.joints.size(), BUF_INIT_NAN);
        hw_math::resize_and_fill(hw_states_effort_, info_.joints.size(), BUF_INIT_NAN);
        hw_math::resize_and_fill(hw_commands_effort_, info_.joints.size(), BUF_INIT_NAN);
        hw_math::resize_and_fill(hw_states_inertia_, INERTIA_MATRIX_FLATTEN_SIZE, BUF_INIT_NAN);
        hw_math::resize_and_fill(hw_button_state_, info_.gpios.size(), BUF_INIT_NAN);

        // ========== 2. 特殊设备逻辑处理 ==========
        // 若启用按键模拟且关节数符合特定配置，初始化最后一个关节（即夹爪）命令为0.0
        if (emulate_button_ && (info_.joints.size() == 4 || info_.joints.size() > 6)) {
            hw_commands_effort_[info_.joints.size() - 1U] = 0.0;
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> FDLeftHardwareInterface::on_export_state_interfaces() {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
        state_if_storage_.clear();

        // 1. 预分配内存，避免 push_back 触发多次重分配
        // 计算总量: 关节数*3 (位置/速度/力矩) + GPIO数 + 惯性矩阵上三角元素(6x6=21)
        const size_t inertia_elements = 21;
        size_t total_size = info_.joints.size() * 3 + info_.gpios.size() + inertia_elements;
        state_interfaces.reserve(total_size);
        state_if_storage_.reserve(total_size);

        // 2. 导出关节状态接口 (位置、速度、力矩)
        for (size_t i = 0; i < info_.joints.size(); i++) {
            const auto& joint_name = info_.joints[i].name;

            // 位置
            auto if_pos = std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
            state_if_storage_.push_back(if_pos);
            state_interfaces.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(if_pos));

            // 速度
            auto if_vel = std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
            state_if_storage_.push_back(if_vel);
            state_interfaces.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(if_vel));

            // 力矩
            auto if_eff = std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]);
            state_if_storage_.push_back(if_eff);
            state_interfaces.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(if_eff));
        }

        // 3. 导出 GPIO 按钮状态
        for (size_t i = 0; i < info_.gpios.size(); i++) {
            const auto& gpio_name = info_.gpios[i].name;
            auto if_btn = std::make_shared<hardware_interface::StateInterface>(
                gpio_name, hardware_interface::HW_IF_POSITION, &hw_button_state_[i]);
            state_if_storage_.push_back(if_btn);
            state_interfaces.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(if_btn));
        }

        // 4. 导出惯性矩阵状态 (6x6 上三角矩阵扁平化存储)
        for (uint row = 0; row < 6; row++) {
            for (uint col = row; col < 6; col++) {
                size_t idx = hw_math::flattened_index_from_triangular_index(row, col);

                // 构建可读性更强的接口名称，例如: "fd_inertia_0_0"
                std::string interface_name = inertia_interface_name_ + "_" +
                    std::to_string(row) + "_" +
                    std::to_string(col);

                auto if_inert = std::make_shared<hardware_interface::StateInterface>(
                    inertia_interface_name_, // 组件名称
                    interface_name, // 接口唯一标识
                    &hw_states_inertia_[idx]);

                state_if_storage_.push_back(if_inert);
                state_interfaces.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(if_inert));
            }
        }

        return state_interfaces;
    }


    std::vector<hardware_interface::CommandInterface::SharedPtr> FDLeftHardwareInterface::on_export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
        command_storage_.clear();
        command_interfaces.reserve(info_.joints.size());

        for (uint i = 0; i < info_.joints.size(); ++i) {
            auto cmd_if = std::make_shared<hardware_interface::CommandInterface>(
                info_.joints[i].name,
                hardware_interface::HW_IF_EFFORT,
                &hw_commands_effort_[i]);

            command_storage_.push_back(cmd_if);
            command_interfaces.push_back(cmd_if);
        }

        return command_interfaces;
    }


    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(LOGGER, "正在激活硬件接口...");

        if (connectToDevice()) {
            RCLCPP_INFO(LOGGER, "硬件设备连接成功");
            return CallbackReturn::SUCCESS;
        } else {
            RCLCPP_ERROR(LOGGER, "硬件设备连接失败");
            return CallbackReturn::ERROR;
        }
    }

    hardware_interface::CallbackReturn FDLeftHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(LOGGER, "正在停用硬件接口...");

        if (disconnectFromDevice()) {
            RCLCPP_INFO(LOGGER, "硬件设备断开成功");
            return CallbackReturn::SUCCESS;
        } else {
            RCLCPP_ERROR(LOGGER, "硬件设备断开失败");
            return CallbackReturn::ERROR;
        }
    }

    hardware_interface::return_type FDLeftHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        int flag = 0; // 错误标志累加器，非0表示API调用异常

        // ---------- 读取位置数据 ----------
        flag += dhdGetPosition(&hw_states_position_[0], &hw_states_position_[1], &hw_states_position_[2], interface_ID_);

        if (!ignore_orientation_ && hw_states_position_.size() > 3) {
            // 读取姿态角 (rad)
            flag += dhdGetOrientationRad(&hw_states_position_[3], &hw_states_position_[4], &hw_states_position_[5], interface_ID_);
        } else if (ignore_orientation_ && hw_states_position_.size() > 3) {
            // 若忽略姿态，置零
            hw_states_position_[3] = 0.0;
            hw_states_position_[4] = 0.0;
            hw_states_position_[5] = 0.0;
        }

        // ---------- 读取夹爪角度 ----------
        if (dhdHasGripper(interface_ID_)) {
            if (hw_states_position_.size() == 4) {
                flag += dhdGetGripperAngleRad(&hw_states_position_[3], interface_ID_);
            } else if (hw_states_position_.size() > 6) {
                flag += dhdGetGripperAngleRad(&hw_states_position_[6], interface_ID_);
            }
        }

        // ---------- 读取线速度 ----------
        flag += dhdGetLinearVelocity(&hw_states_velocity_[0], &hw_states_velocity_[1], &hw_states_velocity_[2], interface_ID_);

        if (!ignore_orientation_ && hw_states_velocity_.size() > 3) {
            // 读取角速度
            flag += dhdGetAngularVelocityRad(&hw_states_velocity_[3], &hw_states_velocity_[4], &hw_states_velocity_[5], interface_ID_);
        } else if (ignore_orientation_ && hw_states_velocity_.size() > 3) {
            // 若忽略姿态，置零
            hw_states_velocity_[3] = 0.0;
            hw_states_velocity_[4] = 0.0;
            hw_states_velocity_[5] = 0.0;
        }

        // ---------- 读取夹爪角速度 ----------
        if (dhdHasGripper(interface_ID_)) {
            if (hw_states_velocity_.size() == 4) {
                flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[3], interface_ID_);
            } else if (hw_states_velocity_.size() > 6) {
                flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[6], interface_ID_);
            }
        }

        // ---------- 读取力/力矩及夹爪力 ----------
        double torque[3];
        double gripper_force;
        flag += dhdGetForceAndTorqueAndGripperForce(
            &hw_states_effort_[0], &hw_states_effort_[1], &hw_states_effort_[2],
            &torque[0], &torque[1], &torque[2],
            &gripper_force,
            interface_ID_
        );

        if (!ignore_orientation_ && hw_states_effort_.size() > 3) {
            // 填充姿态力矩
            hw_states_effort_[3] = torque[0];
            hw_states_effort_[4] = torque[1];
            hw_states_effort_[5] = torque[2];
        } else if (ignore_orientation_ && hw_states_effort_.size() > 3) {
            // 若忽略姿态，置零
            hw_states_effort_[3] = 0.0;
            hw_states_effort_[4] = 0.0;
            hw_states_effort_[5] = 0.0;
        }

        // ---------- 读取夹爪力 ----------
        if (dhdHasGripper(interface_ID_)) {
            if (hw_states_effort_.size() == 4) {
                hw_states_effort_[3] = gripper_force;
            } else if (hw_states_effort_.size() > 6) {
                hw_states_effort_[6] = gripper_force;
            }
        }

        // ---------- 计算并读取 6x6 惯性矩阵 (上三角) ----------
        double inertia_array[6][6];
        double joint_position[DHD_MAX_DOF]; // DHD_MAX_DOF=8, SDK支持最多8自由度，Omega7为7自由度，joint_position[7]保留

        // 进入专家模式以获取底层关节角度
        flag += dhdEnableExpertMode();
        // 获取当前关节角度
        flag += dhdGetJointAngles(joint_position, interface_ID_);
        // 根据关节角度计算 6x6 笛卡尔空间惯性矩阵 (不含夹爪)
        flag += dhdJointAnglesToInertiaMatrix(joint_position, inertia_array, interface_ID_);
        // 退出专家模式
        flag += dhdDisableExpertMode();

        // 将对称矩阵的上三角部分展平存入状态向量
        for (uint row = 0; row < 6; row++) {
            for (uint col = row; col < 6; col++) {
                hw_states_inertia_[hw_math::flattened_index_from_triangular_index(row, col)] = inertia_array[row][col];
            }
        }

        // ---------- 读取按键状态 ----------
        int button_status = dhdGetButton(0, interface_ID_);
        if (button_status == 1) {
            hw_button_state_[0] = 1.0;
        } else if (button_status == 0) {
            hw_button_state_[0] = 0.0;
        } else {
            RCLCPP_ERROR(LOGGER, "按键状态读取异常");
            flag += -1;
        }

        // ---------- 结果判断 ----------
        if (flag >= 0) {
            return hardware_interface::return_type::OK;
        } else {
            RCLCPP_ERROR(LOGGER, "DHD API 调用出现错误");
            return hardware_interface::return_type::ERROR;
        }
    }


    hardware_interface::return_type FDLeftHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        // 检查命令中是否包含 NaN
        bool isNan = false;
        for (const auto& command : hw_commands_effort_) {
            if (std::isnan(command)) {
                isNan = true;
                break;
            }
        }

        // 若无非 NaN 值，则发送命令
        if (!isNan) {
            // 情况1: 带夹爪且自由度 > 6 (6DOF + 夹爪)
            if (dhdHasGripper(interface_ID_) && hw_states_effort_.size() > 6) {
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5],
                    hw_commands_effort_[6], interface_ID_);
            }
            // 情况2: 带腕部但只有4个接口 (3平移 + 夹爪模拟/或特殊配置)
            else if (dhdHasWrist(interface_ID_) && hw_states_effort_.size() == 4) {
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    0.0, 0.0, 0.0, hw_commands_effort_[3], interface_ID_);
            }
            // 情况3: 带腕部且自由度 > 3 (完整 6DOF，无夹爪或夹爪不在控制列表)
            else if (dhdHasWrist(interface_ID_) && hw_states_effort_.size() > 3) {
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5],
                    0, interface_ID_);
            }
            // 情况4: 其他情况 (仅3DOF或未知配置)
            else {
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    0, 0, 0, 0, interface_ID_);
            }
        }
        // 若存在 NaN，发送零力保证安全
        else {
            dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, interface_ID_);
        }

        return hardware_interface::return_type::OK;
    }


    bool FDLeftHardwareInterface::connectToDevice() {
        int major, minor, release, revision;
        // 获取 DHD SDK 版本信息
        dhdGetSDKVersion(&major, &minor, &release, &revision);
        RCLCPP_INFO(
            LOGGER,
            "DHD SDK 版本: %d.%d (release %d / revision %d)",
            major, minor, release, revision);

        bool dhd_open_success = false;
        // 根据序列号打开设备
        if (interface_SN_ >= 0) {
            RCLCPP_INFO(LOGGER, "尝试通过序列号 %d 打开设备...", interface_SN_);
            interface_ID_ = static_cast<char>(dhdOpenSerial(interface_SN_));
            dhd_open_success = (interface_ID_ >= 0);
        }

        if (dhd_open_success) {
            RCLCPP_INFO(LOGGER, "设备名称: %s", dhdGetSystemName(interface_ID_));

            // 获取序列号
            uint16_t serialNumber = 0;
            if (dhdGetSerialNumber(&serialNumber, interface_ID_) < 0) {
                RCLCPP_WARN(LOGGER, "无法获取序列号: %s", dhdErrorGetLastStr());
            } else {
                RCLCPP_INFO(LOGGER, "设备序列号: %d", serialNumber);
            }
            RCLCPP_INFO(LOGGER, "内部接口 ID: %d", interface_ID_);

            // 检测是否具备腕部自由度
            if (dhdHasWrist(interface_ID_)) {
                RCLCPP_INFO(LOGGER, "检测到腕部自由度");
            } else {
                RCLCPP_INFO(LOGGER, "未检测到腕部自由度");
            }

            // 获取当前末端质量
            double current_effector_mass = 0.0;
            if (dhdGetEffectorMass(&current_effector_mass, interface_ID_) == DHD_NO_ERROR) {
                RCLCPP_INFO(LOGGER, "当前末端质量: %.2f g", current_effector_mass * 1000.0);
            } else {
                RCLCPP_WARN(LOGGER, "无法获取末端质量");
            }

            // 设置最大输出力
            double forceMax = 12; // 单位: N
            if (dhdSetMaxForce(forceMax, interface_ID_) < DHD_NO_ERROR) {
                RCLCPP_ERROR(LOGGER, "设置最大力失败");
                disconnectFromDevice();
            }

            // 关闭刹车
            dhdSetBrakes(DHD_OFF, interface_ID_);

            // 启用力反馈
            if (dhdEnableForce(DHD_ON, interface_ID_) < DHD_NO_ERROR) {
                RCLCPP_ERROR(LOGGER, "启用力反馈失败");
                disconnectFromDevice();
                return false;
            }

            // 设置自定义末端质量 (若配置有效)
            if (effector_mass_ > 0.0) {
                RCLCPP_INFO(
                    LOGGER,
                    "更新末端质量: %.2f g -> %.2f g",
                    current_effector_mass * 1000.0,
                    effector_mass_ * 1000.0);
                if (dhdSetEffectorMass(effector_mass_, interface_ID_) < DHD_NO_ERROR) {
                    RCLCPP_ERROR(LOGGER, "设置末端质量失败");
                    disconnectFromDevice();
                    return false;
                }
            }

            // 开启重力补偿
            if (dhdSetGravityCompensation(DHD_ON, interface_ID_) < DHD_NO_ERROR) {
                RCLCPP_ERROR(LOGGER, "开启重力补偿失败");
                disconnectFromDevice();
                return false;
            } else {
                RCLCPP_INFO(LOGGER, "重力补偿已开启");
            }

            RCLCPP_INFO(LOGGER, "设备基本配置完成");

            // 处理按键模拟逻辑
            if (emulate_button_ && !dhdHasGripper(interface_ID_)) {
                RCLCPP_ERROR(LOGGER, "启用按键模拟但设备无夹爪");
            } else if (emulate_button_ && dhdHasGripper(interface_ID_)) {
                RCLCPP_INFO(LOGGER, "设备带有夹爪，启用按键模拟");
                if (dhdEmulateButton(DHD_ON, interface_ID_) < DHD_NO_ERROR) {
                    RCLCPP_ERROR(LOGGER, "启用按键模拟失败");
                    disconnectFromDevice();
                    return false;
                }
                RCLCPP_INFO(LOGGER, "按键模拟功能已激活");
            }

            // 初始化输出为零，防止突变
            if (dhdSetForceAndTorqueAndGripperForce(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                interface_ID_) < DHD_NO_ERROR) {
                RCLCPP_ERROR(LOGGER, "初始化力输出失败");
                disconnectFromDevice();
                return false;
            }

            // 根据硬件能力自动调整忽略姿态标志
            ignore_orientation_ |= !dhdHasWrist(interface_ID_);
            if (ignore_orientation_) {
                RCLCPP_INFO(LOGGER, "因硬件限制，忽略姿态读数");
            }

            // 短暂休眠以确保稳定
            dhdSleep(0.1);
            isConnected_ = true;
            return isConnected_;
        } else {
            RCLCPP_ERROR(LOGGER, "打开设备失败");
            isConnected_ = false;
            return isConnected_;
        }
    }

    bool FDLeftHardwareInterface::disconnectFromDevice() {
        // 停止设备运动
        int hasStopped = -1;
        while (hasStopped < 0) {
            RCLCPP_INFO(LOGGER, "正在停止 DHD 设备...");
            hasStopped = dhdStop(interface_ID_);
            dhdSleep(0.1);
        }

        // 关闭 USB 连接
        int connectionIsClosed = dhdClose(interface_ID_);
        if (connectionIsClosed >= 0) {
            RCLCPP_INFO(LOGGER, "DHD 设备已关闭");
            interface_ID_ = -1;
            isConnected_ = false;
            return true;
        } else {
            RCLCPP_ERROR(LOGGER, "DHD 设备关闭失败");
            return false;
        }
    }
} // namespace fd_left_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fd_left_hardware::FDLeftHardwareInterface, hardware_interface::SystemInterface)
