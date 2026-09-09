#pragma once
#include "raphael_hardware/mtm/fd_base/fd_hardware_base.hpp"

namespace fd_left_hardware{
    class FDLeftHardwareInterface : public fd_hardware_base::FDHardwareBase {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDLeftHardwareInterface);
        ~FDLeftHardwareInterface() override;

        hardware_interface::return_type read(
            const rclcpp::Time& time,
            const rclcpp::Duration& period) override;

        hardware_interface::return_type write(
            const rclcpp::Time& time,
            const rclcpp::Duration& period) override;

    protected:
        char getInterfaceID() override;
        void setInterfaceID(char id) override;

    private:
        char interface_ID_{-1};
    };
} // namespace fd_left_hardware
