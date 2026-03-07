# ROS2 控制系统启动文件
# 这个文件负责启动 ROS2 控制框架的核心组件，包括控制器管理器和各个控制器

# 导入必要的 ROS2 launch 模块
from launch import LaunchDescription  # 启动描述类，用于定义整个启动流程
from launch_ros.actions import Node   # ROS2 节点动作，用于启动 ROS 节点
from launch.actions import TimerAction  # 定时器动作，用于延迟启动
from ament_index_python.packages import get_package_share_directory  # 获取 ROS2 包共享目录
from moveit_configs_utils import MoveItConfigsBuilder  # MoveIt 配置构建器
import os  # 系统路径操作


def generate_launch_description():
    """
    生成 ROS2 控制系统的启动描述

    这个函数创建了完整的 ROS2 控制系统启动配置，包括：
    1. ROS2 控制主节点（controller_manager）
    2. 关节状态广播器（joint_state_broadcaster）
    3. 机器人控制器（robotic_arm_controller）

    这些组件共同构成了机器人控制系统的核心。
    """

    # 创建 MoveIt 配置对象
    # MoveItConfigsBuilder 用于构建 MoveIt 的配置参数
    # 参数：机器人名称 "robotic_arm"，包名 "robotic_config"
    moveit_config = MoveItConfigsBuilder("robotic_arm", package_name="robotic_config").to_moveit_configs()

    # 获取控制器配置文件路径
    # controller_config_dir = get_package_share_directory('robotic_config')
    controller_config_dir = get_package_share_directory('robotic_config')  # 获取 robotic_config 包的共享目录
    controller_config = os.path.join(controller_config_dir, "config", "ros2_controllers.yaml")  # 控制器配置文件路径

    # 创建 ROS2 控制主节点
    # 这个节点是 ROS2 控制系统的核心，管理所有控制器和硬件接口
    ros2_control_node = Node(
        package="controller_manager",        # 包名：控制器管理器
        executable="ros2_control_node",      # 可执行文件：ROS2 控制节点
        parameters=[controller_config,       # 控制器配置文件
                    moveit_config.robot_description,  # 机器人描述（URDF）
                    {'hardware': 'arm_hw'}   # 硬件接口名称
        ],
        output="both"                        # 输出到屏幕和日志
    )

    # 延迟启动关节状态广播器
    # TimerAction 用于延迟启动，确保控制器管理器已经准备好
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,  # 延迟 2 秒启动
        actions=[
            Node(
                package="controller_manager",     # 包名：控制器管理器
                executable="spawner",              # 可执行文件：控制器生成器
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],  # 参数：控制器名和控制器管理器话题
                output="both"                      # 输出到屏幕和日志
            )
        ]
    )

    # 延迟启动机器人控制器
    # 同样使用 TimerAction 延迟启动
    robotic_arm_controller_spawner = TimerAction(
        period=3.0,  # 延迟 3 秒启动，确保关节状态广播器先启动
        actions=[
            Node(
                package="controller_manager",     # 包名：控制器管理器
                executable="spawner",              # 可执行文件：控制器生成器
                arguments=["robotic_arm_controller", "--controller-manager", "/controller_manager"],  # 参数：控制器名和控制器管理器话题
                output="both"                      # 输出到屏幕和日志
            )
        ]
    )

    # 返回启动描述
    # 包含所有需要启动的组件
    return LaunchDescription([
        # ros2_control 主节点
        ros2_control_node,

        # 加载控制器（延迟启动以确保顺序）
        joint_state_broadcaster_spawner,   # 关节状态广播器
        robotic_arm_controller_spawner      # 机器人控制器
    ])
