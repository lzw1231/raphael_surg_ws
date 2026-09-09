#pragma once

#include <hardware_interface/system_interface.hpp>
#include <fd_vendor/fd_sdk.hpp>
#include "raphael_hardware/common/visibility_control.hpp"

namespace fd_hardware_base{
    class RAPHAEL_HARDWARE_PUBLIC FDHardwareBase : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDHardwareBase);

        ~FDHardwareBase() override = default;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface::ConstSharedPtr>
        on_export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface::SharedPtr>
        on_export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override = 0;

        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override = 0;

    protected:
        bool connectToDevice();
        bool disconnectFromDevice();

        virtual char getInterfaceID() = 0;
        virtual void setInterfaceID(char id) = 0;

        std::vector<double> hw_commands_effort_;
        std::vector<double> hw_states_position_;
        std::vector<double> hw_states_velocity_;
        std::vector<double> hw_states_effort_;
        std::vector<double> hw_states_inertia_;
        std::vector<double> hw_button_state_;

        int interface_SN_{-1};
        bool emulate_button_{false};
        std::string inertia_interface_name_;
        double effector_mass_{-1.0};
        bool ignore_orientation_{false};
        bool isConnected_{false};

        static constexpr size_t INERTIA_MATRIX_FLATTEN_SIZE = 21U;
        static constexpr double BUF_INIT_NAN = std::numeric_limits<double>::quiet_NaN();
        static constexpr double DEFAULT_MAX_FORCE{12.0};

        std::vector<hardware_interface::StateInterface::SharedPtr> state_if_storage_;
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_storage_;

        std::string logger_name_{"FDHardwareBase"};

    private:
        bool validateJointInterfaces(const hardware_interface::ComponentInfo& joint);
        bool validateGpioInterfaces(const hardware_interface::ComponentInfo& button);
        void parseHardwareParameters();
    };
} // namespace fd_hardware_base
