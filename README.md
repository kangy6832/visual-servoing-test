# visual-servoing-test

本仓库提供基于 ROS 2 的 PBVS + MoveIt2 联合控制示例，实现了五次多项式轨迹连续处理、速度级控制器以及状态机组织流程。

## 目录结构

- `src/robotic_arm`: 机械臂模型与控制框架代码
  - `include/robotic_arm`: 核心模块头文件
  - `src`: 控制节点与实现
  - `docs`: API 说明与状态机指南

## 快速开始

1. 编译：

```bash
colcon build --packages-select robotic_arm
```

2. 运行 PBVS + MoveIt2 联合控制节点：

```bash
ros2 run robotic_arm pbvs_moveit_node --ros-args --params-file src/robotic_arm/config/pbvs_moveit.yaml
```
