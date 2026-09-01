#pragma once

#include <string>
#include <vector>
#include <controller_interface/controller_interface.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>

using Float64MultiArray = example_interfaces::msg::Float64MultiArray;

namespace my_controller{
    class MyController : public controller_interface::ControllerInterface {
    public:
        MyController();
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    protected:
        std::vector<std::string> joint_names_;
        std::string interface_name_;
        double coefficient_;
        realtime_tools::RealtimeBuffer<std::vector<double>> rt_command_buffer_;

        rclcpp::Subscription<Float64MultiArray>::SharedPtr command_subscriber_;
    }; // class MyController
} // my_controller
