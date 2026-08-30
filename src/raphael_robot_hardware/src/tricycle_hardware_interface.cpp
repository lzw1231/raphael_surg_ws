#include "raphael_robot_hardware/tricycle_hardware_interface.hpp"
#include <cmath>

namespace tricycle_hardware{
    namespace{
        // 位置转换：计数 → rad
        constexpr double POSITION_COUNT_TO_RAD = 2.0 * M_PI / 4096.0;
        // 速度转换：step/s → rad/s
        constexpr double VELOCITY_STEP_TO_RAD_PER_SEC = 0.732 * 2.0 * M_PI / 60.0;
        // 速度转换：rad/s → step/s
        constexpr double VELOCITY_RAD_PER_SEC_TO_STEP = 60.0 / (0.732 * 2.0 * M_PI);
    } // 匿名命名空间确保这些常量仅在本编译单元可见

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        auto ret = hardware_interface::SystemInterface::on_init(params);
        if (ret != hardware_interface::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(get_logger(),
                         "基类 on_init 初始化失败，错误码 %d。 请确认 URDF 的 <hardware> 标签内硬件参数配置正确! ",
                         static_cast<int>(ret));
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto& hw_params = params.hardware_info.hardware_parameters;

        left_motor_id_ = static_cast<u8>(std::stoi(hw_params.at("left_motor_id")));
        right_motor_id_ = static_cast<u8>(std::stoi(hw_params.at("left_motor_id")));
        st3215_port_ = hw_params.at("st3215_port_");

        servo_ = std::make_shared<SMS_STS>();

        RCLCPP_INFO(get_logger(), "Tricycle 硬件初始化成功：左电机ID = %d，右电机ID = %d，串口 = %s", left_motor_id_, right_motor_id_, st3215_port_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        if (!servo_->begin(115200, st3215_port_.c_str())) {
            RCLCPP_ERROR(get_logger(), "串口 %s 打开失败！", st3215_port_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;
        servo_->end();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        // 电机上使能
        servo_->EnableTorque(left_motor_id_, 1);
        servo_->EnableTorque(right_motor_id_, 1);

        // 电机设为速度模式
        servo_->WheelMode(left_motor_id_);
        servo_->WheelMode(right_motor_id_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TricycleHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        // 电机下使能
        servo_->EnableTorque(left_motor_id_, 0);
        servo_->EnableTorque(right_motor_id_, 0);

        return hardware_interface::CallbackReturn::SUCCESS;
    };

    hardware_interface::return_type TricycleHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;
        (void)period;
        // 从电机串口读取左右轮位置和速度原始值，并转换为标准单位（rad 和 rad/s）
        double left_vel = static_cast<double>(servo_->ReadSpeed(left_motor_id_)) * VELOCITY_STEP_TO_RAD_PER_SEC;
        double right_vel = static_cast<double>(servo_->ReadSpeed(right_motor_id_)) * VELOCITY_STEP_TO_RAD_PER_SEC;
        double left_pos = static_cast<double>(servo_->ReadPos(left_motor_id_)) * POSITION_COUNT_TO_RAD;
        double right_pos = static_cast<double>(servo_->ReadPos(right_motor_id_)) * POSITION_COUNT_TO_RAD;

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
        double left_cmd_step_fp = get_command("base_left_wheel_joint/velocity") * VELOCITY_RAD_PER_SEC_TO_STEP;
        double right_cmd_step_fp = get_command("base_right_wheel_joint/velocity") * VELOCITY_RAD_PER_SEC_TO_STEP;

        // 写入电机
        servo_->WriteSpe(left_motor_id_, static_cast<int16_t>(left_cmd_step_fp));
        servo_->WriteSpe(right_motor_id_, static_cast<int16_t>(right_cmd_step_fp));

        return hardware_interface::return_type::OK;
    };
} // namespace tricycle_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tricycle_hardware::TricycleHardwareInterface, hardware_interface::SystemInterface)
