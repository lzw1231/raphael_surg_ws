import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取功能包共享目录路径，定位 URDF、RViz 配置及控制器配置文件
    robot_desc_file_path = os.path.join(
        get_package_share_directory("raphael_robot_description"),
        "urdf",
        "raphael_robot.urdf.xacro"
    )
    robot_controllers_yaml_path = os.path.join(
        get_package_share_directory("raphael_robot_bringup"),
        "config",
        "raphael_robot_controllers.yaml"
    )
    rviz_config_file_path = os.path.join(
        get_package_share_directory("raphael_robot_description"),
        "rviz",
        "urdf_config.rviz"
    )

    # 机器人状态发布节点：解析 Xacro 文件并发布 robot_description 参数
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', robot_desc_file_path]),
                value_type=str
            )
        }]
    )

    # 控制器管理器节点：加载并管理 ros2_control 定义的硬件接口与控制器
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers_yaml_path]
    )

    # 激活关节状态广播器：用于发布非受控关节（如被动轮或传感器支架）的状态
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    # 激活三轮小车差分驱动控制器：负责底盘运动控制
    tricycle_diff_driver_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["tricycle_diff_driver_controller"]
    )

    # 激活蛇形驱动控制器：用于控制蛇形驱动的运动模式
    # snake_driver_controller_node = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=["snake_driver_controller"]
    # )

    # 激活蛇形驱动控制器_模拟：用于在仿真环境中测试蛇形驱动控制器
    snake_driver_controller_mock_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["snake_driver_controller_mock"]
    )

    # RViz 可视化节点：加载指定的配置文件以展示机器人模型
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file_path]
    )

    # 返回包含所有节点的启动描述
    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_node,
        tricycle_diff_driver_controller_node,
        # snake_driver_controller_node,
        snake_driver_controller_mock_node,
        rviz_node
    ])
