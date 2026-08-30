#include "raphael_robot_hardware/snake_hardware_interface.hpp"

namespace snake_hardware{
    namespace{
        // 位置转换：计数 → rad
        constexpr double POSITION_COUNT_TO_RAD = 2.0 * M_PI / 4095;
        // 位置转换：rad → 计数
        constexpr double RAD_TO_POSITION_COUNT = 4095 / (2.0 * M_PI);
        // 舵机中立位置
        constexpr s16 POSITION_ZERO = 2047;
        // 速度转换：step/s → rad/s
        constexpr double VELOCITY_STEP_TO_RAD_PER_SEC = 2.0 * M_PI / 4095;
        // 速度转换：rad/s → step/s
        constexpr double VELOCITY_RAD_PER_SEC_TO_STEP = 4095 / (2.0 * M_PI);
    } // 匿名命名空间,确保这些常量仅在本编译单元可见


    hardware_interface::CallbackReturn SnakeHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        if (auto ret = hardware_interface::SystemInterface::on_init(params); ret != hardware_interface::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(get_logger(),
                         "基类 on_init 初始化失败，错误码 %d。 请确认 URDF <hardware> 标签内硬件参数配置正确! ",
                         static_cast<int>(ret));
            return ret;
        }

        const auto& hw_params = params.hardware_info.hardware_parameters;

        motor_ids_[S1_06_07] = static_cast<u8>(std::stoi(hw_params.at("s1_motor_06_07")));
        motor_ids_[S1_07_08] = static_cast<u8>(std::stoi(hw_params.at("s1_motor_07_08")));
        motor_ids_[S1_08_09] = static_cast<u8>(std::stoi(hw_params.at("s1_motor_08_09")));
        motor_ids_[S1_09B_10] = static_cast<u8>(std::stoi(hw_params.at("s1_motor_09b_10")));
        motor_ids_[S1_10_11] = static_cast<u8>(std::stoi(hw_params.at("s1_motor_10_11")));

        st3215_port_ = hw_params.at("st3215_port");
        baud_ = static_cast<int>(std::stoi(hw_params.at("baud")));

        sms_sts_ = std::make_shared<SMS_STS>();

        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn SnakeHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        if (!sms_sts_->begin(baud_, st3215_port_.c_str())) {
            RCLCPP_ERROR(get_logger(), "串口初始化失败: 无法连接设备 %s", st3215_port_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "[ST3215] 串口已初始化: %s @ %d bps", st3215_port_.c_str(), baud_);

        // 舵机 Ping 检测
        for (size_t i = 0; i < motor_ids_.size(); ++i) {
            const u8 id = motor_ids_[i];
            const u8 ping_id = sms_sts_->Ping(id);
            if (motor_ids_[i] != sms_sts_->Ping(id)) {
                RCLCPP_ERROR(get_logger(), "snake舵机Ping检测失败: 期望ID %u, 实际返回 %u",
                             static_cast<unsigned>(id),
                             static_cast<unsigned>(ping_id));
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SnakeHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;
        (void)previous_state;

        // sms_sts_->end();
        // RCLCPP_INFO(get_logger(), "串口通信链路已关闭");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SnakeHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        for (size_t i = 0; i < motor_ids_.size(); ++i) {
            sms_sts_->ServoMode(motor_ids_[i]);
        }

        RCLCPP_INFO(get_logger(), "snake舵机已设为位置模式");

        // 准备5个电机的目标位置、速度、加速度数组
        std::array<s16, 5> positions;
        std::array<u16, 5> speeds;
        std::array<u8, 5> accs;

        positions.fill(POSITION_ZERO);
        speeds.fill(3400);
        accs.fill(50);

        // 舵机同步运行
        sms_sts_->SyncWritePosEx(motor_ids_.data(), motor_ids_.size(), positions.data(), speeds.data(), accs.data());

        rclcpp::sleep_for(std::chrono::milliseconds(3000));

        RCLCPP_INFO(get_logger(), "snake舵机已到达中立位2047...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SnakeHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        // 准备5个电机的目标位置、速度、加速度数组
        std::array<s16, 5> positions;
        std::array<u16, 5> speeds;
        std::array<u8, 5> accs;

        positions.fill(0);
        speeds.fill(3400);
        accs.fill(50);

        // 舵机同步运行
        sms_sts_->SyncWritePosEx(motor_ids_.data(), motor_ids_.size(), positions.data(), speeds.data(), accs.data());

        rclcpp::sleep_for(std::chrono::milliseconds(3000));

        RCLCPP_INFO(get_logger(), "snake舵机已回机械零位...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SnakeHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;
        (void)period;

        auto pos_06_07 = static_cast<double>((sms_sts_->ReadPos(motor_ids_[S1_06_07]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);
        auto pos_07_08 = static_cast<double>((sms_sts_->ReadPos(motor_ids_[S1_07_08]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);
        auto pos_08_09 = static_cast<double>((sms_sts_->ReadPos(motor_ids_[S1_08_09]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);
        auto pos_09b_10 = static_cast<double>((sms_sts_->ReadPos(motor_ids_[S1_09B_10]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);
        auto pos_10_11 = static_cast<double>((sms_sts_->ReadPos(motor_ids_[S1_10_11]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);

        // 更新状态接口
        set_state("s1_link06_link07_joint/position", pos_06_07);
        set_state("s1_link07_link08_joint/position", pos_07_08);
        set_state("s1_link08_link09_joint/position", pos_08_09);
        set_state("s1_link09b_link10_joint/position", pos_09b_10);
        set_state("s1_link10_link11_joint/position", pos_10_11);

        // 输出舵机位置
        RCLCPP_INFO(get_logger(),
                    "舵机_STATE: ID[%d]=%.4f, ID[%d]=%.4f, ID[%d]=%.4f, ID[%d]=%.4f, ID[%d]=%.4f",
                    motor_ids_[S1_06_07], pos_06_07,
                    motor_ids_[S1_07_08], pos_07_08,
                    motor_ids_[S1_08_09], pos_08_09,
                    motor_ids_[S1_09B_10], pos_09b_10,
                    motor_ids_[S1_10_11], pos_10_11
        );

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SnakeHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;
        (void)period;
        struct JointCmd {
            std::string name;
            double& value;
        };

        // 变量定义
        double s1_06_07_pos_raw = get_command("s1_link06_link07_joint/position");
        double s1_07_08_pos_raw = get_command("s1_link07_link08_joint/position");
        double s1_08_09_pos_raw = get_command("s1_link08_link09_joint/position");
        double s1_09B_10_pos_raw = get_command("s1_link09b_link10_joint/position");
        double s1_10_11_pos_raw = get_command("s1_link10_link11_joint/position");

        std::vector<JointCmd> cmd_list = {
            {"s1_link06_link07_joint", s1_06_07_pos_raw},
            {"s1_link07_link08_joint", s1_07_08_pos_raw},
            {"s1_link08_link09_joint", s1_08_09_pos_raw},
            {"s1_link09b_link10_joint", s1_09B_10_pos_raw},
            {"s1_link10_link11_joint", s1_10_11_pos_raw},
        };

        bool has_error = false;
        for (auto& item : cmd_list) {
            if (std::isnan(item.value)) {
                RCLCPP_ERROR(get_logger(), "[NaN DETECT] joint: %s , raw value=%f", item.name.c_str(), item.value);
                has_error = true;
            }
        }

        if (has_error) {
            RCLCPP_WARN(get_logger(), "Abort motor write due to NaN command(s)");
            return hardware_interface::return_type::OK;
        }


        // servo_->WritePosEx(motor_ids_[S1_06_07], static_cast<u16>(s1_06_07_pos), 3400, 50);
        // servo_->WritePosEx(motor_ids_[S1_07_08], static_cast<u16>(s1_07_08_pos), 3400, 50);
        // servo_->WritePosEx(motor_ids_[S1_08_09], static_cast<u16>(s1_08_09_pos), 3400, 50);
        // servo_->WritePosEx(motor_ids_[S1_09B_10], static_cast<u16>(s1_09B_10_pos), 3400, 50);
        // servo_->WritePosEx(motor_ids_[S1_10_11], static_cast<u16>(s1_10_11_pos), 3400, 50);


        return hardware_interface::return_type::OK;
    }
} //namespace snake_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(snake_hardware::SnakeHardwareInterface, hardware_interface::SystemInterface)




