#include "raphael_hardware/mtm/fd_left/fd_left_hardware.hpp"
#include "raphael_hardware/common/math_utils.hpp"

namespace fd_left_hardware{
    FDLeftHardwareInterface::~FDLeftHardwareInterface() {
        (void)disconnectFromDevice();
    }

    char FDLeftHardwareInterface::getInterfaceID() {
        return interface_ID_;
    }

    void FDLeftHardwareInterface::setInterfaceID(char id) {
        interface_ID_ = id;
    }

    hardware_interface::return_type FDLeftHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        auto logger = rclcpp::get_logger(logger_name_);
        int flag = 0;
        char dev_id = getInterfaceID();

        flag += dhdGetPosition(&hw_states_position_[0], &hw_states_position_[1], &hw_states_position_[2], dev_id);
        if (!ignore_orientation_ && hw_states_position_.size() > 3) {
            flag += dhdGetOrientationRad(&hw_states_position_[3], &hw_states_position_[4], &hw_states_position_[5], dev_id);
        } else if (ignore_orientation_ && hw_states_position_.size() > 3) {
            hw_states_position_[3] = 0.0;
            hw_states_position_[4] = 0.0;
            hw_states_position_[5] = 0.0;
        }

        if (dhdHasGripper(dev_id)) {
            if (hw_states_position_.size() == 4) {
                flag += dhdGetGripperAngleRad(&hw_states_position_[3], dev_id);
            } else if (hw_states_position_.size() > 6) {
                flag += dhdGetGripperAngleRad(&hw_states_position_[6], dev_id);
            }
        }

        flag += dhdGetLinearVelocity(&hw_states_velocity_[0], &hw_states_velocity_[1], &hw_states_velocity_[2], dev_id);
        if (!ignore_orientation_ && hw_states_velocity_.size() > 3) {
            flag += dhdGetAngularVelocityRad(&hw_states_velocity_[3], &hw_states_velocity_[4], &hw_states_velocity_[5], dev_id);
        } else if (ignore_orientation_ && hw_states_velocity_.size() > 3) {
            hw_states_velocity_[3] = 0.0;
            hw_states_velocity_[4] = 0.0;
            hw_states_velocity_[5] = 0.0;
        }

        if (dhdHasGripper(dev_id)) {
            if (hw_states_velocity_.size() == 4) {
                flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[3], dev_id);
            } else if (hw_states_velocity_.size() > 6) {
                flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[6], dev_id);
            }
        }

        double torque[3];
        double gripper_force;
        flag += dhdGetForceAndTorqueAndGripperForce(
            &hw_states_effort_[0], &hw_states_effort_[1], &hw_states_effort_[2],
            &torque[0], &torque[1], &torque[2],
            &gripper_force,
            dev_id
        );

        if (!ignore_orientation_ && hw_states_effort_.size() > 3) {
            hw_states_effort_[3] = torque[0];
            hw_states_effort_[4] = torque[1];
            hw_states_effort_[5] = torque[2];
        } else if (ignore_orientation_ && hw_states_effort_.size() > 3) {
            hw_states_effort_[3] = 0.0;
            hw_states_effort_[4] = 0.0;
            hw_states_effort_[5] = 0.0;
        }

        if (dhdHasGripper(dev_id)) {
            if (hw_states_effort_.size() == 4) {
                hw_states_effort_[3] = gripper_force;
            } else if (hw_states_effort_.size() > 6) {
                hw_states_effort_[6] = gripper_force;
            }
        }

        double inertia_array[6][6];
        double joint_position[DHD_MAX_DOF];
        flag += dhdEnableExpertMode();
        flag += dhdGetJointAngles(joint_position, dev_id);
        flag += dhdJointAnglesToInertiaMatrix(joint_position, inertia_array, dev_id);
        flag += dhdDisableExpertMode();

        for (uint row = 0; row < 6; row++) {
            for (uint col = row; col < 6; col++) {
                hw_states_inertia_[hw_math::flattened_index_from_triangular_index(row, col)] = inertia_array[row][col];
            }
        }

        int button_status = dhdGetButton(0, dev_id);
        if (button_status == 1) {
            hw_button_state_[0] = 1.0;
        } else if (button_status == 0) {
            hw_button_state_[0] = 0.0;
        } else {
            RCLCPP_ERROR(logger, "按键状态读取异常");
            flag += -1;
        }

        if (flag >= 0) {
            return hardware_interface::return_type::OK;
        } else {
            RCLCPP_ERROR(logger, "DHD API 调用出现错误");
            return hardware_interface::return_type::ERROR;
        }
    }

    hardware_interface::return_type FDLeftHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        char dev_id = getInterfaceID();
        bool isNan = false;
        for (auto& command : hw_commands_effort_) {
            if (std::isnan(command)) {
                isNan = true;
                break;
            }
        }

        if (!isNan) {
            if (dhdHasGripper(dev_id) && hw_states_effort_.size() > 6) {
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5],
                    hw_commands_effort_[6], dev_id);
            } else if (dhdHasWrist(dev_id) && hw_states_effort_.size() == 4) {
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    0.0, 0.0, 0.0, hw_commands_effort_[3], dev_id);
            } else if (dhdHasWrist(dev_id) && hw_states_effort_.size() > 3) {
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5],
                    0, dev_id);
            } else {
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    0, 0, 0, 0, dev_id);
            }
        } else {
            dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, dev_id);
        }
        return hardware_interface::return_type::OK;
    }
} // namespace fd_left_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fd_left_hardware::FDLeftHardwareInterface, hardware_interface::SystemInterface)
