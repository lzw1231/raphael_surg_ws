#pragma once

#include <hardware_interface/system_interface.hpp>
#include "raphael_hardware/common/visibility_control.hpp"

/**
 * @brief Force Dimension 左侧设备硬件接口
 * @details 实现 ROS2 HardwareInterface 用于控制力反馈设备/读取状态
 *          支持位置/速度/力矩反馈以及GPIO按钮状态读取
 */
namespace fd_left_hardware{
    class FDLeftHardwareInterface : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDLeftHardwareInterface);

        /**
         * @brief 析构函数
         */
        ~FDLeftHardwareInterface() override;

        /**
         * @brief 初始化硬件接口参数
         * @param params 硬件组件接口参数
         * @return 回调返回状态
         */
        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

        /**
         * @brief 配置阶段回调，用于验证参数或预分配资源
         * @param previous_state 之前的生命周期状态
         * @return 回调返回状态
         */
        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        /**
         * @brief 导出状态接口供控制器读取
         * @return 状态接口智能指针列表
         */
        RAPHAEL_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

        /**
         * @brief 导出命令接口供控制器写入
         * @return 命令接口智能指针列表
         */
        RAPHAEL_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

        /**
         * @brief 激活阶段回调，通常用于连接硬件或启动通信
         * @param previous_state 之前的生命周期状态
         * @return 回调返回状态
         */
        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        /**
         * @brief 去激活阶段回调，通常用于断开连接或停止通信
         * @param previous_state 之前的生命周期状态
         * @return 回调返回状态
         */
        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        /**
         * @brief 从硬件读取数据并更新状态接口
         * @param time 当前时间
         * @param period 周期时长
         * @return 返回类型
         */
        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;

        /**
         * @brief 将命令接口数据写入硬件
         * @param time 当前时间
         * @param period 周期时长
         * @return 返回类型
         */
        RAPHAEL_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

    private:
        std::vector<double> hw_commands_effort_; ///< [Cmd] 力矩/力命令缓冲区
        std::vector<double> hw_states_position_; ///< [State] 关节位置状态
        std::vector<double> hw_states_velocity_; ///< [State] 关节速度状态
        std::vector<double> hw_states_effort_; ///< [State] 关节力矩/力状态
        std::vector<double> hw_states_inertia_; ///< [State] 惯性矩阵数据
        std::vector<double> hw_button_state_; ///< [State] GPIO按钮状态

        char interface_ID_ = -1; ///< ID
        int interface_SN_ = -1; ///< 序列号
        bool emulate_button_ = false; ///< 是否模拟按钮输入
        std::string inertia_interface_name_; ///< 惯性接口名称
        double effector_mass_ = -1.0; ///< 末端执行器质量
        bool ignore_orientation_ = false; ///< 是否忽略姿态
        bool isConnected_ = false; ///< 连接状态标志
        bool device_disconnected_ = false; ///< 设备断开标志
        size_t num_joints_ = 0U; ///< 关节数量
        size_t num_gpios_ = 0U; ///< GPIO数量

        static constexpr size_t INERTIA_MATRIX_FLATTEN_SIZE = 21U; ///< 惯性矩阵展平后的大小 (6x6对称矩阵的上三角部分)
        static constexpr double BUF_INIT_NAN = std::numeric_limits<double>::quiet_NaN(); ///< 缓冲区初始化NaN值

        std::vector<hardware_interface::StateInterface::SharedPtr> state_if_storage_; ///< 状态接口存储
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_storage_; ///< 命令接口存储

        /**
         * @brief 连接到 Force Dimension 设备
         * @return 成功返回true，失败返回false
         */
        bool connectToDevice();

        /**
         * @brief 断开与设备的连接
         * @return 成功返回true，失败返回false
         */
        bool disconnectFromDevice();
    }; // class FDLeftHardwareInterface
} // namespace fd_left_hardware
