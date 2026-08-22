import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_desc_file_path = os.path.join(get_package_share_directory("raphael_robot_description"), "urdf", "raphael_robot_urdf.xacro")
    rviz_config_file_path = os.path.join(get_package_share_directory("raphael_robot_description"), "config", "urdf_config.rviz")
    robot_controllers_path = os.path.join(get_package_share_directory("raphael_robot_bringup"), "config", "raphael_robot_rviz.launch.py")

    robot_desc = ParameterValue(Command([' xacro ', robot_desc_file_path]), value_type=str)
