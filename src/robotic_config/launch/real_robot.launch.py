"""
真实机器人视觉伺服系统启动文件

目的：
----------
启动真实机器人的完整视觉伺服（Visual Servoing）系统，协调机器人驱动、
视觉感知、运动规划和控制系统，实现基于视觉反馈的机器人精确控制。

与仿真环境的区别：
-----------------
1. 使用真实机器人硬件接口（非仿真）
2. 配置为不使用仿真时间（use_sim_time: False）
3. 包含真实的相机坐标变换标定
4. 直接与机器人控制器通信

具体功能：
-------------------
1. 机器人硬件驱动：
   - 启动robot_driver节点，与真实机器人控制器通信
   - 发送关节控制指令，接收机器人状态反馈

2. 视觉传感器标定与坐标变换：
   - 发布相机到机器人关节1（link1）的静态坐标变换
   - 变换参数：位置 (0.04023, -0.20514, 0.26134) 米
   - 旋转：绕X、Y、Z轴各旋转90度（1.570796弧度）
   - 确保视觉坐标系与机器人坐标系的正确对齐

3. 机器人建模与运动规划：
   - 加载机器人URDF/SRDF描述文件
   - 启动MoveIt运动规划框架
   - 提供路径规划、碰撞检测、逆运动学求解

4. 控制系统集成：
   - 启动ROS2控制系统（ros2_control）
   - 管理关节控制器、状态发布器

5. 可视化与监控：
   - 启动RViz可视化工具
   - 实时显示机器人状态、规划路径、相机数据

使用场景：
--------
- 真实机器人的视觉伺服实验与研究
- 基于视觉的精确抓取与放置操作
- 机器人手眼协调控制
- 视觉引导的轨迹跟踪

启动的组件及顺序：
-----------------
1. static_tf - 静态坐标变换（为后续组件提供正确的坐标系）
2. robot_description_launch_py - 机器人描述（建立机器人模型）
3. move_group - MoveIt运动规划组（基于模型进行规划）
4. rviz_show - RViz可视化（监控系统状态）
5. joint_driver - 机器人驱动（控制真实硬件）
6. start_ros_control - ROS控制系统（管理控制接口）

注意事项：
--------
1. 确保真实机器人硬件已正确连接并上电
2. 相机设备需要提前完成内参和外参标定
3. 坐标变换参数需根据相机实际安装位置调整
4. 机器人驱动节点需要相应的硬件接口支持
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    """
    生成真实机器人视觉伺服系统的启动描述

    核心任务：
    ----------
    1. 初始化真实机器人硬件接口，建立与机器人控制器的通信
    2. 配置视觉传感器坐标变换，建立相机与机器人的空间关系
    3. 加载机器人运动学和动力学模型
    4. 启动运动规划系统，提供路径规划和逆运动学求解
    5. 集成控制系统，管理关节控制器和状态反馈
    6. 启动可视化界面，实时监控系统状态

    详细组件说明：
    --------------
    - joint_driver: 真实机器人驱动节点
      * 功能：与机器人控制器通信，发送控制指令，接收状态反馈
      * 配置：use_sim_time=False（适用于真实硬件）

    - static_tf: 静态坐标变换发布器
      * 功能：发布相机到link1的固定坐标变换
      * 参数：位置偏移 (0.04023, -0.20514, 0.26134) 米
      * 参数：旋转角度 (1.570796, -1.570796, 1.570796) 弧度

    - robot_description_launch_py: 机器人描述启动文件
      * 功能：加载机器人URDF/SRDF描述文件
      * 作用：建立机器人的运动学和动力学模型

    - move_group: MoveIt运动规划组
      * 功能：提供运动规划、碰撞检测、逆运动学求解
      * 作用：将视觉伺服目标转换为关节空间轨迹

    - rviz_show: RViz可视化工具
      * 功能：实时显示机器人状态、规划路径、相机数据
      * 作用：系统监控和调试

    - start_ros_control: ROS2控制系统
      * 功能：管理关节控制器、状态发布器
      * 作用：提供标准的控制接口

    返回值：
    -------
    LaunchDescription
        包含所有启动节点和launch文件的ROS2启动描述对象。
        组件按依赖关系顺序排列，确保系统正确初始化。

    异常：
    -----
    FileNotFoundError
        如果依赖的launch文件不存在

    PackageNotFoundError
        如果robotic_config包未找到
    """


    # 获取robotic_config包的共享目录路径
    # 使用ament_index_python工具获取ROS2包的安装路径
    moveit_config_dir = get_package_share_directory("robotic_config")
    
    # 构建launch文件目录的完整路径
    # 所有相关的launch文件都位于包的launch子目录中
    launch_dir = os.path.join(moveit_config_dir, "launch")

    # 创建真实机器人驱动节点
    # 该节点负责与真实机器人硬件控制器通信
    # 关键配置：use_sim_time=False表示使用真实时间，适用于真实硬件
    joint_driver = Node(
            package="robot_driver",          # 机器人驱动包名
            executable="robot_driver",       # 可执行文件名称
            output="screen",                 # 输出到屏幕，便于调试
            parameters=[{
                "use_sim_time": False        # 不使用仿真时间，适用于真实硬件
            }]
        )
    
    
    
    # 创建静态坐标变换发布节点
    # 发布相机到机器人关节1（link1）的固定坐标变换
    # 此变换对于视觉伺服至关重要，确保视觉坐标系与机器人坐标系对齐
    # 变换参数说明：
    #   - 位置：x=0.04023m, y=-0.20514m, z=0.26134m
    #   - 旋转：绕X轴旋转90度，绕Y轴旋转-90度，绕Z轴旋转90度（弧度制）
    static_tf = Node(
        package='tf2_ros',                              # tf2坐标变换包
        executable='static_transform_publisher',        # 静态变换发布器
        arguments=[
            '0.0', '0.0', '0.0',          # 位置：x, y, z (米)
            '0.0', '0.0', '0.0',       # 旋转：roll, pitch, yaw (弧度)
            'link5', 'camera_link'                      # 源坐标系 -> 目标坐标系（link6是末端执行器）
        ]
    )


    # 构建各个launch文件的完整路径
    # 这些launch文件由MoveIt Setup Assistant生成，包含机器人配置
    robot_description_launch = os.path.join(launch_dir, "rsp.launch.py")          # 机器人状态发布器
    move_group_launch = os.path.join(launch_dir, "move_group.launch.py")          # MoveIt运动规划组
    rviz_launch = os.path.join(launch_dir, "moveit_rviz.launch.py")               # RViz可视化配置
    ros_control_launch = os.path.join(launch_dir, "bringup.launch.py")            # ROS2控制系统

    # 包含ROS2控制系统launch文件
    # 启动ros2_control框架，管理关节控制器和硬件接口
    start_ros_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_control_launch)
    )

    # 包含机器人描述launch文件
    # 加载机器人URDF/SRDF描述，发布机器人状态和变换
    robot_description_launch_py = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch)
    )
    
    # 包含MoveIt运动规划组launch文件
    # 启动MoveIt框架，提供运动规划、逆运动学、碰撞检测等功能
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch)
    )

    # 包含RViz可视化launch文件
    # 启动RViz工具，可视化机器人状态、规划路径和传感器数据
    rviz_show = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch)
    )


    # 返回完整的启动描述
    # 组件按依赖关系顺序排列，确保系统正确初始化：
    # 1. 静态坐标变换（为其他组件提供正确的坐标系）
    # 2. ROS控制系统（管理控制接口，必须先启动）
    # 3. 机器人描述（建立机器人模型）
    # 4. 运动规划组（基于模型进行规划，依赖控制系统）
    # 5. 机器人驱动（控制真实硬件，在控制系统之后启动）
    # 6. 可视化界面（监控系统状态）
    return LaunchDescription([
        static_tf,                     # 静态坐标变换发布器
        start_ros_control,             # ROS2控制系统（必须先启动）
        robot_description_launch_py,   # 机器人描述
        move_group,                    # MoveIt运动规划组
        joint_driver,                  # 机器人驱动节点（在控制系统后启动）
        rviz_show                      # RViz可视化
    ])
