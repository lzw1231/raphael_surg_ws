#pragma once


#include <memory>
#include <vector>
#include <string>


#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/macros.hpp>

#include "raphael_hardware/visibility_control.hpp"


namespace fd_left_hardware{
    class FDLeftHardwareInterface : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDLeftHardwareInterface);
        ~FDLeftHardwareInterface() override;

        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;

        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

    private:
        std::vector<double> hw_commands_effort_;
        std::vector<double> hw_states_position_;
        std::vector<double> hw_states_velocity_;
        std::vector<double> hw_states_effort_;
        std::vector<double> hw_states_inertia_; // upper-triangular matrix
        std::vector<double> hw_button_state_;

        char interface_ID_ = -1;
        int interface_SN_ = -1;
        bool emulate_button_ = false;
        std::string inertia_interface_name_;
        double effector_mass_ = -1.0;
        bool ignore_orientation_ = false;
        bool isConnected_ = false;
        bool device_disconnected_{false};

        /**
         * @brief 初始化与力反馈设备的USB通信链路
         * @warning 调用本接口后，设备将使能力矩输出，需确保设备处于安全工作状态
         * @return true: 通信初始化成功; false: USB通信建立失败
         */
        bool connectToDevice();

        /**
         * @brief 终止与力反馈设备的USB通信链路
         * @details 详细错误信息输出参考重载接口 disconnectFromDevice(std_msgs::msg::String& str)
         * @return true: 通信断开成功; false: USB链路关闭异常
         */
        bool disconnectFromDevice();
    }; // class FDLeftHardwareInterface
} // namespace fd_left_hardware
