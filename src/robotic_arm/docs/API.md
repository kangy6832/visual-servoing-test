# API 说明文档

## 总览
本包提供基于 ROS 2 的 PBVS + MoveIt2 联合控制框架，包含：

- **运动学求解器接口**：为上层规划与视觉伺服提供统一的 FK/IK 调用入口。
- **五次多项式轨迹模块**：在 MoveIt2 规划轨迹后进行连续性处理。
- **速度级控制器**：支持基于关节速度的指令发布。
- **PBVS 控制器**：通过位姿误差生成速度级指令。
- **状态机模块**：管理规划、执行、PBVS 模式的切换。

## 命名空间
所有核心类均位于 `robotic_arm` 命名空间下，子模块如下：

- `robotic_arm::trajectory`
- `robotic_arm::kinematics`
- `robotic_arm::control`
- `robotic_arm::planning`
- `robotic_arm::state_machine`

## 轨迹模块

### `QuinticTrajectory`
用于对离散轨迹点进行五次多项式插值，输出连续的位姿/速度/加速度。

**头文件**：`robotic_arm/trajectory/quintic_trajectory.hpp`

**主要接口**：

- `setWaypoints(times, positions, velocities, accelerations)`
  - 输入：时间戳、位置/速度/加速度序列。
  - 说明：会自动构建每段的五次多项式系数。
- `sample(time)`
- `sampleVelocity(time)`
- `sampleAcceleration(time)`

## 运动学模块

### `KinematicsSolver`
用于提供 FK/IK 接口占位实现（可对接 KDL / MoveIt IK 插件）。

**头文件**：`robotic_arm/kinematics/kinematics_solver.hpp`

**主要接口**：

- `setJointLimits(min_limits, max_limits)`
- `solveIK(target_pose, seed)`
- `solveFK(joint_positions)`

## 控制模块

### `VelocityController`
发布速度级关节指令（`trajectory_msgs/msg/JointTrajectory`）。

**头文件**：`robotic_arm/control/velocity_controller.hpp`

**主要接口**：

- `setJointNames(joint_names)`
- `setCommandTopic(topic)`
- `sendVelocityCommand(velocities, duration_sec)`

### `PBVSController`
根据当前末端位姿与目标位姿生成速度级 Twist 指令。

**头文件**：`robotic_arm/control/pbvs_controller.hpp`

**主要接口**：

- `setGains(linear_gain, angular_gain)`
- `setMaxVelocity(linear_limit, angular_limit)`
- `computeTwistCommand(current, target)`

## 规划模块

### `MoveItPlanner`
封装 MoveIt2 规划接口。

**头文件**：`robotic_arm/planning/moveit_planner.hpp`

**主要接口**：

- `setPlannerId(planner_id)`
- `setPlanningTime(seconds)`
- `planToPose(target_pose)`
- `planToJointGoal(joint_goal)`

## 状态机模块

### `StateMachine`
轻量级状态机，用于 PBVS + MoveIt2 联合流程的状态管理。

**头文件**：`robotic_arm/state_machine/state_machine.hpp`

**主要接口**：

- `addState(name, on_enter, on_tick, on_exit)`
- `addTransition(from, to, guard)`
- `setInitialState(name)`
- `tick()`

## 示例节点

### `pbvs_moveit_node`
启动后负责：

1. 使用 MoveIt2 规划至目标位姿。
2. 使用五次多项式对轨迹平滑。
3. 通过速度级控制器执行轨迹与 PBVS 修正。

**订阅话题**：

- `/target_pose` (`geometry_msgs/msg/PoseStamped`)
- `/joint_states` (`sensor_msgs/msg/JointState`)

**发布话题**：

- `/joint_velocity_controller/joint_trajectory`

**可配置参数**：

- `joint_names`
- `planner_id`
- `planning_time`
- `pbvs.linear_gain`
- `pbvs.angular_gain`
- `pbvs.linear_limit`
- `pbvs.angular_limit`
