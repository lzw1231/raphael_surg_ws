#pragma once

#include <hardware_interface/system_interface.hpp>

namespace fd_utils{
    /**
     * @brief 校验单个关节的 ros2_control command/state interface
     * @param joint ComponentInfo 关节信息
     * @param logger rclcpp logger引用，用于打印FATAL日志
     * @return true 校验通过；false 校验失败，内部打印日志
     */
    bool validate_joint_interfaces(const hardware_interface::ComponentInfo& joint, rclcpp::Logger& logger);

    /**
     * @brief 校验单个GPIO button state interface
     * @param button ComponentInfo gpio信息
     * @param logger rclcpp logger引用
     * @return true 校验通过；false 校验失败，内部打印日志
     */
    bool validate_gpio_interfaces(const hardware_interface::ComponentInfo& button, rclcpp::Logger& logger);

    /**
     * @brief 批量校验所有joint + gpio接口
     * @param info HardwareInfo
     * @param logger rclcpp logger引用
     * @return true 全部校验通过；false 任意一项失败
     */
    bool validate_hardware_interfaces(const hardware_interface::HardwareInfo& info, rclcpp::Logger& logger);

    /**
     * @brief 硬件参数结构体，一次性返回所有解析出来的参数
     */
    struct FDHardwareParams {
        int interface_sn{-1};
        bool emulate_button{false};
        std::string inertia_interface_name{"fd_inertia"};
        double effector_mass{-1.0};
        bool ignore_orientation{false};
    };

    /**
     * @brief 从HardwareInfo读取硬件参数，返回参数结构体
     * @param info HardwareInfo
     * @param logger rclcpp logger引用
     * @return FDHardwareParams 解析结果
     * @note 参数非法字符串会直接抛出std异常，不捕获，fail-fast
     */
    FDHardwareParams load_hardware_parameters(const hardware_interface::HardwareInfo& info, rclcpp::Logger& logger);
} // namespace fd_utils
