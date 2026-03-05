# 视觉伺服机器人控制系统

## 项目概述

这是一个基于ROS2的机器人视觉伺服控制系统，用于实现机器人手臂的视觉引导抓取和放置任务。系统包含控制器、任务管理、运动规划和视觉伺服等功能模块。

## 项目结构

```
visual_servoing/
├── src/
│   ├── control_pack/          # 控制包 - 机器人任务管理和运动规划
│   │   ├── include/control_pack/
│   │   │   ├── motion_generator_base.hpp
│   │   │   ├── robot_pose_polynomial.hpp
│   │   │   ├── robotic_task.hpp
│   │   │   ├── trajectory_executor.hpp
│   │   │   └── velocity_ik_generator.hpp
│   │   ├── lib/
│   │   ├── src/
│   │   │   ├── robot_pose_polynomial.cpp
│   │   │   ├── robotic_task.cpp
│   │   │   ├── trajectory_executor.cpp
│   │   │   └── velocity_ik_generator.cpp
│   │   ├── API_Documentation.md
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── controller/            # 控制器 - ROS2控制器实现
│   │   ├── include/controller/
│   │   │   └── visual_servoing_controller.hpp
│   │   ├── src/
│   │   │   └── visual_servoing_controller.cpp
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── plugins.xml
│   ├── robot_driver/          # 机器人驱动
│   │   ├── include/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── robot_interfaces/      # 机器人接口定义
│       ├── action/
│       │   └── Catch.action
│       ├── include/
│       ├── msg/
│       │   ├── Image.msg
│       │   ├── Joint.msg
│       │   └── Robot.msg
│       ├── CMakeLists.txt
│       └── package.xml
├── .gitignore
├── .windsurfignore
├── 99-usb-robot.rules         # USB机器人规则
├── AIR_PUMP_IMPROVEMENTS_SUMMARY.md  # 气泵改进总结
├── README.md
├── test_air_pump.py           # 气泵测试脚本
├── 状态机实现总结.md            # 状态机实现总结
├── 路径规划实现指导.md          # 路径规划实现指导
├── build/                     # 构建目录 (自动生成)
├── install/                   # 安装目录 (自动生成)
└── log/                       # 日志目录 (自动生成)
```

## 附加文档

本项目包含以下附加文档：

- `AIR_PUMP_IMPROVEMENTS_SUMMARY.md`：气泵改进总结
- `状态机实现总结.md`：状态机实现总结
- `路径规划实现指导.md`：路径规划实现指导
- `API_Documentation.md`：控制包API文档

## 功能特性

1. **视觉伺服控制**：基于视觉反馈的机器人末端执行器控制
2. **任务管理**：支持移动、抓取、放置等多种机器人任务
3. **运动规划**：使用MoveIt进行路径规划和碰撞检测
4. **实时控制**：ROS2控制器接口实现实时关节控制
5. **多项式轨迹**：五次多项式轨迹生成平滑运动

## 依赖项

### 系统依赖
- Ubuntu 22.04或更高版本
- ROS2 Humble或Iron
- CMake 3.16+
- C++17兼容编译器

### ROS2包依赖
- `rclcpp`
- `rclcpp_action`
- `controller_interface`
- `moveit_core`
- `moveit_ros_planning_interface`
- `tf2_ros`
- `geometry_msgs`
- `visualization_msgs`

## 构建说明

### 1. 环境设置
```bash
# 设置ROS2环境
source /opt/ros/humble/setup.bash

# 创建工作空间
mkdir -p ~/visual_servoing_ws/src
cd ~/visual_servoing_ws/src
```

### 2. 克隆项目
```bash
# 克隆项目到工作空间
git clone <repository-url> visual_servoing
```

### 3. 安装依赖
```bash
# 进入工作空间根目录
cd ~/visual_servoing_ws

# 安装ROS2依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 4. 构建项目
```bash
# 使用colcon构建
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 或者使用CMake直接构建
cd visual_servoing
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## 使用方法

### 1. 启动控制器
```bash
# 设置环境
source ~/visual_servoing_ws/install/setup.bash

# 启动控制器节点
ros2 launch controller visual_servoing_controller.launch.py
```

### 2. 运行任务系统
```bash
# 启动任务管理节点
ros2 run control_pack robotic_task_node
```

### 3. 发送抓取任务
```bash
# 通过ROS2 action发送抓取任务
ros2 action send_goal /catch_target robot_interfaces/action/Catch "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, action_type: 2}"
```

## 配置说明

### 机器人参数配置
配置文件位于：`src/robotic_arm/config/joint_names_robotic_arm.yaml`

### 控制器参数
- 关节名称：joint1-joint6
- 控制频率：100Hz
- 速度缩放因子：0.4
- 加速度缩放因子：0.3

## 任务状态机

系统实现以下任务状态：
1. `IDLE` - 空闲状态
2. `MOVE_TO_READY_CATCH_POINT` - 移动到预备抓取位置
3. `MOVE_TO_CATCH_POINT` - 移动到抓取位置
4. `CATCH_TARGET` - 抓取目标
5. `VISUAL_SERVOING` - 视觉伺服
6. `MOVE_TO_RELEASE_POINT` - 移动到释放位置
7. `RELEASE_TARGET` - 释放目标
8. `MOVE_TO_IDLE_POINT` - 移动到空闲位置

## 开发指南

### 代码风格
- 使用C++17标准
- 遵循ROS2编码规范
- 使用Doxygen格式注释

### 添加新功能
1. 在相应包中创建头文件和源文件
2. 更新CMakeLists.txt添加编译目标
3. 更新package.xml添加依赖
4. 编写测试用例

## 故障排除

### 常见问题
1. **TF变换错误**：检查相机和基座标系之间的TF树
2. **运动规划失败**：调整规划参数或检查碰撞物体
3. **控制器加载失败**：检查插件配置和依赖项

### 调试工具
```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 查看节点图
rqt_graph

# 查看TF变换
ros2 run tf2_ros tf2_echo base_link camera_link
```

## 许可证

本项目采用MIT许可证。详见LICENSE文件。

## 贡献指南

欢迎提交Issue和Pull Request来改进本项目。

## 联系方式

如有问题或建议，请通过以下方式联系：
- 项目仓库：<repository-url>
- Issue跟踪：<repository-url>/issues