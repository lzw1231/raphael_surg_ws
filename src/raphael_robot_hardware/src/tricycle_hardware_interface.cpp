#include "raphael_robot_hardware/tricycle_hardware_interface.hpp"
#include <cmath>

namespace tricycle_hardware{
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

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        if (auto ret = hardware_interface::SystemInterface::on_init(params); ret != hardware_interface::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(get_logger(),
                         "基类 on_init 初始化失败，错误码 %d。 请确认 URDF <hardware> 标签内硬件参数配置正确! ",
                         static_cast<int>(ret));
            return ret;
        }

        const auto& hw_params = params.hardware_info.hardware_parameters;

        left_motor_id_ = static_cast<u8>(std::stoi(hw_params.at("left_motor_id")));
        right_motor_id_ = static_cast<u8>(std::stoi(hw_params.at("right_motor_id")));
        st3215_port_ = hw_params.at("st3215_port");
        baud_ = std::stoi(hw_params.at("baud"));

        sms_sts_ = std::make_shared<SMS_STS>();

        RCLCPP_INFO(get_logger(), "Tricycle 硬件初始化成功：左电机ID = %d，右电机ID = %d，串口 = %s", left_motor_id_, right_motor_id_, st3215_port_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        if (!sms_sts_->begin(baud_, st3215_port_.c_str())) {
            RCLCPP_ERROR(get_logger(), "串口 %s 打开失败！", st3215_port_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "[ST3215] 串口已初始化: %s @ %d bps", st3215_port_.c_str(), baud_);

        u8 left_ping_id = sms_sts_->Ping(left_motor_id_);
        u8 right_ping_id = sms_sts_->Ping(right_motor_id_);

        if (left_motor_id_ != left_ping_id) {
            RCLCPP_ERROR(get_logger(), "舵机Ping检测失败: 期望ID %u, 实际返回 %u",
                         static_cast<unsigned>(left_motor_id_),
                         static_cast<unsigned>(left_ping_id));
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (right_motor_id_ != right_ping_id) {
            RCLCPP_ERROR(get_logger(), "舵机Ping检测失败: 期望ID %u, 实际返回 %u",
                         static_cast<unsigned>(right_motor_id_),
                         static_cast<unsigned>(right_ping_id));
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;
        sms_sts_->end();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        // 电机设为速度模式
        sms_sts_->WheelMode(left_motor_id_);
        sms_sts_->WheelMode(right_motor_id_);

        RCLCPP_INFO(get_logger(), "舵机已设为速度模式");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;


        return hardware_interface::CallbackReturn::SUCCESS;
    };

    hardware_interface::return_type TricycleHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;
        (void)period;
        // 从电机串口读取左右轮位置和速度原始值，并转换为标准单位（rad 和 rad/s）
        double left_vel = static_cast<double>(sms_sts_->ReadSpeed(left_motor_id_)) * VELOCITY_STEP_TO_RAD_PER_SEC;
        double right_vel = static_cast<double>(sms_sts_->ReadSpeed(right_motor_id_)) * VELOCITY_STEP_TO_RAD_PER_SEC;
        double left_pos = static_cast<double>(sms_sts_->ReadPos(left_motor_id_)) * POSITION_COUNT_TO_RAD;
        double right_pos = static_cast<double>(sms_sts_->ReadPos(right_motor_id_)) * POSITION_COUNT_TO_RAD;

        // 写入实时共享内存，供控制器读取
        set_state("base_left_wheel_joint/velocity", left_vel);
        set_state("base_right_wheel_joint/velocity", right_vel);
        set_state("base_left_wheel_joint/position", left_pos);
        set_state("base_right_wheel_joint/position", right_pos);

        return hardware_interface::return_type::OK;
    };

    hardware_interface::return_type TricycleHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;
        (void)period;


        // 从命令接口缓存获取控制器下发的角速度 (rad/s)，并转换为 setp/秒
        double left_vel_step = get_command("base_left_wheel_joint/velocity") * VELOCITY_RAD_PER_SEC_TO_STEP;
        double right_vel_step = get_command("base_right_wheel_joint/velocity") * VELOCITY_RAD_PER_SEC_TO_STEP;

        // 写入电机
        sms_sts_->WriteSpe(left_motor_id_, static_cast<int16_t>(left_vel_step));
        sms_sts_->WriteSpe(right_motor_id_, static_cast<int16_t>(right_vel_step));

        return hardware_interface::return_type::OK;
    };
} // namespace tricycle_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tricycle_hardware::TricycleHardwareInterface, hardware_interface::SystemInterface)
