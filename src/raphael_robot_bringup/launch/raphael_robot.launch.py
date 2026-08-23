import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_desc_file_path = os.path.join(get_package_share_directory("raphael_robot_description"), "urdf", "raphael_robot_urdf.xacro")
    rviz_config_file_path = os.path.join(get_package_share_directory("raphael_robot_description"), "config", "urdf_config.rviz")
    robot_controllers_yaml_path = os.path.join(get_package_share_directory("raphael_robot_bringup"), "config", "raphael_robot_controllers.yaml")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command([' xacro ', robot_desc_file_path]), value_type=str)}]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers_yaml_path]
    )

    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    diff_driver_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diff_driver_controller"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_node,
        diff_driver_controller_node,
        rviz_node
    ])
