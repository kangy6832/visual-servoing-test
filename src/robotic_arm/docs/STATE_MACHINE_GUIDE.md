# 状态机书写指南

## 目标
为了保证 PBVS + MoveIt2 联合控制流程的清晰性与可维护性，建议通过状态机组织流程逻辑。

## 设计原则

1. **单一职责**：每个状态只做一件事，如“规划”、“执行”、“视觉伺服校正”。
2. **显式过渡条件**：过渡条件（guard）必须可追踪、可打印。
3. **避免隐式全局状态**：所有影响决策的变量建议集中在上下文结构体中。
4. **可恢复性**：每个状态应支持失败重试或回退策略。
5. **可测试性**：将关键行为拆为独立函数，便于单元测试。

## 推荐状态划分

- `Idle`：等待目标。
- `Plan`：调用 MoveIt2 规划轨迹。
- `Smooth`：五次多项式平滑。
- `Execute`：下发速度/位置指令。
- `PBVS`：视觉伺服闭环修正。
- `Error`：处理异常与回退。

## 转换规则模板

| From | To | Guard | Description |
| ---- | --- | ----- | ----------- |
| Idle | Plan | has_target | 收到目标位姿 |
| Plan | Smooth | plan_ok | 规划成功 |
| Smooth | Execute | smooth_ok | 轨迹平滑完成 |
| Execute | PBVS | near_target | 进入视觉伺服 |
| * | Error | error_detected | 任何异常 |

## 建议的上下文结构

```cpp
struct ControlContext {
  bool has_target{false};
  bool plan_ok{false};
  bool smooth_ok{false};
  bool error_detected{false};
  geometry_msgs::msg::Pose target_pose;
  moveit_msgs::msg::RobotTrajectory planned_traj;
  robotic_arm::trajectory::QuinticTrajectory quintic_traj;
};
```

## 示例伪代码

```cpp
state_machine.addState("Plan", onEnterPlan, onTickPlan, onExitPlan);
state_machine.addTransition("Plan", "Smooth", [&]() { return ctx.plan_ok; });
state_machine.addTransition("Plan", "Error", [&]() { return ctx.error_detected; });
```

## 常见问题

- **状态机卡死**：确认 guard 条件是否被更新。
- **重复进入状态**：检查是否在 `on_tick` 内重复触发 `changeState`。
- **异常未处理**：统一在 `Error` 状态记录并重置上下文。
