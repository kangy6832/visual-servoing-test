# Control Pack API 文档

本文档详细描述了 `control_pack` 包中三个核心模块的 API 接口和使用方法。

## 目录

- [robot_pose_polynomial.hpp](#robot_pose_polynomialhpp) - 五次多项式轨迹插值器
- [trajectory_executor.hpp](#trajectory_executorhpp) - 轨迹执行器（KDL/动力学/Action）
- [velocity_ik_generator.hpp](#velocity_ik_generatorhpp) - KDL 运动学计算库

---

## robot_pose_polynomial.hpp

### 概述

该模块提供五次多项式轨迹插值功能，用于生成平滑的关节空间轨迹，保证位置、速度、加速度的连续性。

**主要特性：**
- C² 连续性（位置、速度、加速度连续）
- 支持多段连续轨迹管理
- 实时安全的插值计算
- 适用于机器人关节空间轨迹规划

### 类：QuinticParam

五次多项式轨迹插值器，用于单段轨迹的插值计算。

#### 构造函数

```cpp
QuinticParam();
```

#### 公共方法

##### set_param()

```cpp
void set_param(
    const double t0, const double t1, 
    const double p0, const double v0, const double a0, 
    const double pt, const double v1, const double at
);
```

**功能：** 设置五次多项式的边界条件参数

**参数：**
- `t0` - 起始时间 [s]
- `t1` - 终止时间 [s]
- `p0` - 起始位置 [rad 或 m]
- `v0` - 起始速度 [rad/s 或 m/s]
- `a0` - 起始加速度 [rad/s² 或 m/s²]
- `pt` - 终止位置 [rad 或 m]
- `v1` - 终止速度 [rad/s 或 m/s]
- `at` - 终止加速度 [rad/s² 或 m/s²]

**数学模型：**
```
p(t) = a·t⁵ + b·t⁴ + c·t³ + d·t² + e·t + f
```

**边界条件：**
```
p(t0) = p0,  ṗ(t0) = v0,  p̈(t0) = a0
p(t1) = pt,  ṗ(t1) = v1,  p̈(t1) = at
```

##### get_position()

```cpp
double get_position(const double t);
```

**功能：** 计算给定时间的位置值

**参数：**
- `t` - 当前时间 [s]

**返回值：** 时间 t 对应的位置 [rad 或 m]

##### get_velocity()

```cpp
double get_velocity(const double t);
```

**功能：** 计算给定时间的速度值

**参数：**
- `t` - 当前时间 [s]

**返回值：** 时间 t 对应的速度 [rad/s 或 m/s]

##### get_acceleration()

```cpp
double get_acceleration(const double t);
```

**功能：** 计算给定时间的加速度值

**参数：**
- `t` - 当前时间 [s]

**返回值：** 时间 t 对应的加速度 [rad/s² 或 m/s²]

#### 使用示例

```cpp
// 创建插值器
QuinticParam joint_trajectory;

// 设置边界条件：从 0 位置平滑移动到 π/2，速度和加速度从 0 到 0
joint_trajectory.set_param(
    0.0, 2.0,    // 时间：0s 到 2s
    0.0, 0.0, 0.0,  // 起始：位置=0, 速度=0, 加速度=0
    M_PI/2, 0.0, 0.0  // 终止：位置=π/2, 速度=0, 加速度=0
);

// 在 t=1.0s 时获取状态
double pos = joint_trajectory.get_position(1.0);      // ≈ π/4
double vel = joint_trajectory.get_velocity(1.0);       // ≈ 最大速度
double acc = joint_trajectory.get_acceleration(1.0);    // ≈ 0
```

---

### 类：ContinuousTrajectory

多段连续轨迹管理器，用于管理和执行由多个轨迹点组成的连续轨迹。

#### 构造函数

```cpp
explicit ContinuousTrajectory();
```

#### 公共方法

##### get_target()

```cpp
bool get_target(
    const rclcpp::Time& time, 
    trajectory_msgs::msg::JointTrajectoryPoint& output
);
```

**功能：** 获取指定时间的期望轨迹点

**参数：**
- `time` [in] - 当前时间（系统时间）
- `output` [out] - 输出的关节期望状态（位置/速度/加速度）

**返回值：**
- `true` - 成功计算目标点
- `false` - 轨迹已结束或轨迹非法

**工作流程：**
1. 计算相对于轨迹起始时间的时间偏移
2. 判断是否需要切换到下一段轨迹
3. 使用当前段的五次多项式计算状态
4. 填充输出消息的 6 个关节数据

##### start_track()

```cpp
void start_track(rclcpp::Time now);
```

**功能：** 启动轨迹跟踪，设置基准时间

**参数：**
- `now` - 当前系统时间，作为轨迹 time_from_start 的零点

**作用：**
- 设置轨迹起始时间
- 重置当前段索引
- 清空初始化标志

##### set_trajectory()

```cpp
void set_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
```

**功能：** 设置要执行的关节轨迹

**参数：**
- `trajectory` - ROS2 标准关节轨迹消息

**要求：**
- 轨迹至少包含 2 个点
- 每个点包含位置、速度、加速度信息
- 时间戳单调递增

#### 使用示例

```cpp
// 创建轨迹管理器
ContinuousTrajectory trajectory_manager;

// 设置轨迹（从 MoveIt 或其他规划器获取）
trajectory_msgs::msg::JointTrajectory planned_trajectory;
// ... 填充轨迹数据 ...

trajectory_manager.set_trajectory(planned_trajectory);

// 开始执行
trajectory_manager.start_track(node->now());

// 控制循环中获取目标点
trajectory_msgs::msg::JointTrajectoryPoint target_point;
if (trajectory_manager.get_target(node->now(), target_point)) {
    // 使用 target_point.positions, .velocities, .accelerations
    // 发送到控制器...
} else {
    // 轨迹执行完成
}
```

---

## trajectory_executor.hpp

### 概述

该模块将原本混合在 `robot_pose_polynomial` 中的 KDL、动力学和 ROS2 Action 功能分离出来，提供专门的轨迹执行服务。

**主要职责：**
1. 从 URDF 初始化 KDL 运动学链和动力学求解器
2. 提供关节空间动力学计算接口
3. 处理 FollowJointTrajectory Action 的回调
4. 管理轨迹执行状态（执行中/取消/完成）

### 类：TrajectoryExecutor

轨迹执行器，整合 KDL 运动学、动力学计算和 ROS2 Action 服务。

#### 构造函数

```cpp
explicit TrajectoryExecutor(const rclcpp::Node::SharedPtr node);
```

**参数：**
- `node` - ROS2 节点共享指针，用于日志和参数客户端

#### 公共方法

##### initFromURDF()

```cpp
bool initFromURDF(const std::string& urdf_xml);
```

**功能：** 从 URDF 字符串初始化 KDL 和动力学求解器

**参数：**
- `urdf_xml` - URDF 字符串描述

**返回值：**
- `true` - 初始化成功
- `false` - 初始化失败

**初始化步骤：**
1. 从 URDF 字符串解析 KDL Tree
2. 提取 base_link 到 tool0 的运动链
3. 创建动力学求解器（考虑重力）
4. 预分配 KDL 数组内存

**注意：** 必须在使用 `dynamicCalc()` 之前调用

##### dynamicCalc()

```cpp
Eigen::Vector<double, 6> dynamicCalc();
```

**功能：** 计算关节空间动力学

**返回值：** 6 维动力学向量（关节力矩）

**计算公式：**
```
τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q)
```

其中：
- `M(q)` - 惯性矩阵
- `C(q,q̇)` - 科里奥利和离心力项
- `G(q)` - 重力项

**前置条件：**
- 已调用 `initFromURDF()`
- 已设置 `q_kdl`, `dq_kdl`, `ddq_kdl` 的值

**警告：** 如果未初始化动力学求解器，返回零向量

#### Action 服务器回调

##### handle_goal()

```cpp
rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid, 
    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal
);
```

**功能：** 处理新的 Action 目标

**参数：**
- `uuid` - 目标唯一标识符
- `goal` - FollowJointTrajectory 目标指针

**返回值：** `GoalResponse` - 接受或拒绝目标

**当前实现：** 总是接受并执行目标

**TODO：** 添加目标验证逻辑（时间约束、关节限制等）

##### handle_cancel()

```cpp
rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
);
```

**功能：** 处理 Action 取消请求

**参数：**
- `goal_handle` - 要取消的目标句柄

**返回值：** `CancelResponse` - 接受或拒绝取消

**当前实现：** 总是接受取消请求

**TODO：** 实现优雅的轨迹停止逻辑

##### handle_accepted()

```cpp
void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
);
```

**功能：** 处理已接受的 Action 目标

**参数：**
- `goal_handle` - 目标句柄

**作用：** 在新目标被接受后调用，用于启动执行线程

**TODO：** 实现实际的轨迹执行逻辑

#### 状态管理

**公共成员变量：**
- `bool is_execut_trajectory` - 是否正在执行轨迹
- `bool cancle_execut` - 是否请求取消执行
- `bool finished_execut` - 轨迹执行是否已完成

#### 使用示例

```cpp
// 创建轨迹执行器
auto executor = std::make_shared<TrajectoryExecutor>(node);

// 从参数服务器获取 URDF
auto urdf_string = node->get_parameter("robot_description").as_string();

// 初始化
if (!executor->initFromURDF(urdf_string)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize trajectory executor");
    return;
}

// 设置关节状态（从硬件读取）
executor->q_kdl.data = current_joint_positions;
executor->dq_kdl.data = current_joint_velocities;
executor->ddq_kdl.data = desired_joint_accelerations;

// 计算动力学
auto joint_torques = executor->dynamicCalc();

// 使用计算结果发送到硬件控制器
// ...
```

---

## velocity_ik_generator.hpp

### 概述

基于 KDL 和 Eigen 的机械臂运动学计算库，提供完整的正向/逆向运动学、雅可比矩阵计算和奇异性分析功能。

**主要特性：**
- 完整的 6DOF 机械臂运动学解决方案
- 实时安全的数值计算
- 奇异性检测和处理
- 支持多种初始化方式（DH参数、URDF、标准6DOF）

### 常量定义

```cpp
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEFAULT_SINGULARITY_THRESHOLD = 1e-4;
constexpr double DEFAULT_MAX_VELOCITY = 10.0;           // m/s
constexpr double DEFAULT_MAX_ANGULAR_VELOCITY = 10.0;    // rad/s
constexpr double POSITION_TOLERANCE = 1e-6;
constexpr double ORIENTATION_TOLERANCE = 1e-6;
```

### 异常类

#### DimensionMismatchException

维度不匹配异常，当输入向量维度与期望不符时抛出。

```cpp
class DimensionMismatchException : public std::runtime_error {
public:
    explicit DimensionMismatchException(const std::string& msg);
    DimensionMismatchException(int expected, int actual, const std::string& context);
};
```

#### KinematicSolverException

运动学求解异常，当 IK 求解失败时抛出。

```cpp
class KinematicSolverException : public std::runtime_error {
public:
    explicit KinematicSolverException(const std::string& msg);
};
```

#### JointLimitException

关节限位异常，当关节角度超出限制时抛出。

```cpp
class JointLimitException : public std::runtime_error {
public:
    explicit JointLimitException(const std::string& msg);
};
```

#### SingularityException

奇异性异常，当机械臂接近奇异位形时抛出。

```cpp
class SingularityException : public std::runtime_error {
public:
    explicit SingularityException(const std::string& msg);
};
```

### 数据结构

#### DHParameters

DH 参数结构，用于描述机械臂连杆参数。

```cpp
struct DHParameters {
    double a;      // 连杆长度 [m]
    double alpha;  // 连杆扭角 [rad]
    double d;      // 连杆偏距 [m]
    double theta;  // 关节角度 [rad]
};
```

#### JointLimits

关节限位配置。

```cpp
struct JointLimits {
    Eigen::VectorXd lower;  // 下限 [rad 或 m]
    Eigen::VectorXd upper;  // 上限 [rad 或 m]
    Eigen::VectorXd velocity_limit;    // 速度限制 [rad/s 或 m/s]
    Eigen::VectorXd acceleration_limit; // 加速度限制 [rad/s² 或 m/s²]
};
```

#### EndEffectorPose

末端执行器位姿描述。

```cpp
struct EndEffectorPose {
    Eigen::Vector3d position;     // 位置 [m]
    Eigen::Quaterniond orientation; // 姿态（四元数）
};
```

#### CartesianTwist

笛卡尔空间速度（旋量）。

```cpp
struct CartesianTwist {
    Eigen::Vector3d linear;   // 线速度 [m/s]
    Eigen::Vector3d angular;  // 角速度 [rad/s]
};
```

#### CartesianAcceleration

笛卡尔空间加速度。

```cpp
struct CartesianAcceleration {
    Eigen::Vector3d linear;   // 线加速度 [m/s²]
    Eigen::Vector3d angular;  // 角加速度 [rad/s²]
};
```

#### SingularityAnalysis

奇异性分析结果。

```cpp
struct SingularityAnalysis {
    double manipulability_index;    // 可操作性指标
    double condition_number;        // 条件数
    double minimum_singular_value;  // 最小奇异值
    bool is_near_singularity;       // 是否接近奇异
    std::string singularity_type;   // 奇异类型描述
};
```

### 类：RobotArmKinematics

机械臂运动学计算类，提供完整的运动学解决方案。

#### 构造函数

```cpp
explicit RobotArmKinematics(size_t num_joints = 6);
```

**参数：**
- `num_joints` - 关节数量（默认 6）

#### 初始化方法

##### initFromDHParams()

```cpp
bool initFromDHParams(const std::vector<DHParameters>& dh_params);
```

**功能：** 使用 DH 参数初始化机械臂链

**参数：**
- `dh_params` - DH 参数列表

**返回值：**
- `true` - 初始化成功
- `false` - 初始化失败

##### initStandard6DOF()

```cpp
bool initStandard6DOF();
```

**功能：** 使用标准 6DOF 机械臂参数初始化

**返回值：**
- `true` - 初始化成功
- `false` - 初始化失败

##### loadFromURDF()

```cpp
bool loadFromURDF(
    const std::string& urdf_path, 
    const std::string& base_link = "base_link", 
    const std::string& tip_link = "tool0"
);
```

**功能：** 从 URDF 文件加载机械臂模型

**参数：**
- `urdf_path` - URDF 文件路径
- `base_link` - 基座连接名称（默认 "base_link"）
- `tip_link` - 末端链接名称（默认 "tool0"）

**返回值：**
- `true` - 加载成功
- `false` - 加载失败

#### 正向运动学

##### computeForwardKinematics()

```cpp
EndEffectorPose computeForwardKinematics(const Eigen::VectorXd& joint_positions);
```

**功能：** 计算正向运动学（末端位姿）

**参数：**
- `joint_positions` - 关节位置 [rad 或 m]

**返回值：** 末端执行器位姿

**异常：**
- `DimensionMismatchException` - 维度不匹配
- `NotInitializedException` - 未初始化

##### computeForwardKinematicsMatrix()

```cpp
Eigen::Matrix4d computeForwardKinematicsMatrix(const Eigen::VectorXd& joint_positions);
```

**功能：** 计算正向运动学（4×4 齐次变换矩阵）

**参数：**
- `joint_positions` - 关节位置

**返回值：** 4×4 齐次变换矩阵

#### 逆向运动学

##### inverseKinematics()

```cpp
Eigen::VectorXd inverseKinematics(
    const EndEffectorPose& target_pose,
    const Eigen::VectorXd& initial_guess
);
```

**功能：** 数值法逆运动学（位姿）

**参数：**
- `target_pose` - 目标末端位姿
- `initial_guess` - 初始关节角

**返回值：** 求解得到的关节角

**异常：**
- `DimensionMismatchException` - 维度不匹配
- `KinematicSolverException` - IK 求解失败
- `JointLimitException` - 超出关节限位

##### inverseKinematics() (带成功标志)

```cpp
bool inverseKinematics(
    const EndEffectorPose& target_pose,
    const Eigen::VectorXd& initial_guess,
    Eigen::VectorXd& solution
);
```

**功能：** 带成功标志的逆运动学（不抛异常版本，适合实时）

**参数：**
- `target_pose` - 目标末端位姿
- `initial_guess` - 初始关节角
- `solution` [out] - 输出解

**返回值：** `true` - 成功

#### 雅可比矩阵

##### computeJacobian()

```cpp
Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& joint_positions);
```

**功能：** 计算雅可比矩阵

**参数：**
- `joint_positions` - 关节位置

**返回值：** 6×n 雅可比矩阵

**异常：** `DimensionMismatchException` - 维度不匹配

##### computeJacobianDerivative()

```cpp
Eigen::MatrixXd computeJacobianDerivative(
    const Eigen::VectorXd& joint_positions, 
    const Eigen::VectorXd& joint_velocities
);
```

**功能：** 计算雅可比矩阵的导数

**参数：**
- `joint_positions` - 关节位置
- `joint_velocities` - 关节速度

**返回值：** 6×n 雅可比矩阵导数

#### 速度计算

##### forwardVelocityKinematics()

```cpp
CartesianTwist forwardVelocityKinematics(
    const Eigen::VectorXd& joint_positions, 
    const Eigen::VectorXd& joint_velocities
);
```

**功能：** 正向速度运动学：从关节速度计算末端速度

**参数：**
- `joint_positions` - 关节位置
- `joint_velocities` - 关节速度

**返回值：** 末端速度（Twist）

**异常：**
- `DimensionMismatchException` - 维度不匹配
- `SingularityException` - 奇异位形

##### inverseVelocityKinematics()

```cpp
Eigen::VectorXd inverseVelocityKinematics(
    const Eigen::VectorXd& joint_positions, 
    const CartesianTwist& desired_velocity
);
```

**功能：** 逆向速度运动学：从末端速度计算关节速度

**参数：**
- `joint_positions` - 关节位置
- `desired_velocity` - 期望末端速度

**返回值：** 关节速度

##### dampedLeastSquaresInverseVelocity()

```cpp
Eigen::VectorXd dampedLeastSquaresInverseVelocity(
    const Eigen::VectorXd& joint_positions, 
    const CartesianTwist& desired_velocity, 
    double damping_factor = -1.0
);
```

**功能：** 使用阻尼最小二乘法计算逆速度

**参数：**
- `joint_positions` - 关节位置
- `desired_velocity` - 期望末端速度
- `damping_factor` - 阻尼因子（-1 表示自动计算）

**返回值：** 关节速度

#### 加速度计算

##### forwardAccelerationKinematics()

```cpp
CartesianAcceleration forwardAccelerationKinematics(
    const Eigen::VectorXd& joint_positions, 
    const Eigen::VectorXd& joint_velocities, 
    const Eigen::VectorXd& joint_accelerations
);
```

**功能：** 正向加速度运动学：从关节加速度计算末端加速度

**参数：**
- `joint_positions` - 关节位置
- `joint_velocities` - 关节速度
- `joint_accelerations` - 关节加速度 [rad/s²]

**返回值：** 末端加速度

#### 奇异性分析

##### analyzeSingularity()

```cpp
SingularityAnalysis analyzeSingularity(const Eigen::VectorXd& joint_positions);
```

**功能：** 分析雅可比矩阵的奇异值

**参数：**
- `joint_positions` - 关节位置

**返回值：** 奇异值分析结果

##### isNearSingularity()

```cpp
bool isNearSingularity(const Eigen::VectorXd& joint_positions);
```

**功能：** 检查是否接近奇异位形

**参数：**
- `joint_positions` - 关节位置

**返回值：** `true` - 接近奇异位形

##### computeManipulabilityIndex()

```cpp
double computeManipulabilityIndex(const Eigen::VectorXd& joint_positions);
```

**功能：** 计算可操作性指标

**参数：**
- `joint_positions` - 关节位置

**返回值：** 可操作性指标

#### 工具方法

##### validateInput()

```cpp
template<typename Derived>
void validateInput(
    const Eigen::MatrixBase<Derived>& data, 
    const std::string& context
) const;
```

**功能：** 验证输入数据的有效性

**参数：**
- `data` - 输入数据
- `context` - 上下文描述

**异常：** `DimensionMismatchException` - 维度不匹配

##### checkFinite()

```cpp
template<typename Derived>
void checkFinite(
    const Eigen::MatrixBase<Derived>& data,  
    const std::string& context
) const;
```

**功能：** 检查数据是否为有限值

**参数：**
- `data` - 输入数据
- `context` - 上下文描述

**异常：** `std::runtime_error` - 包含无效值

#### 状态查询

##### isInitialized()

```cpp
bool isInitialized() const;
```

**返回值：** `true` - 已初始化

##### getNumJoints()

```cpp
size_t getNumJoints() const;
```

**返回值：** 关节数量

##### getLastJacobian()

```cpp
const Eigen::MatrixXd& getLastJacobian() const;
```

**返回值：** 最后计算的雅可比矩阵

##### getStatusMessage()

```cpp
const std::string& getStatusMessage() const;
```

**返回值：** 状态消息

#### 使用示例

```cpp
// 创建 6DOF 机械臂运动学求解器
RobotArmKinematics kinematics(6);

// 使用标准参数初始化
if (!kinematics.initStandard6DOF()) {
    std::cerr << "Failed to initialize kinematics" << std::endl;
    return -1;
}

// 设置关节限位
JointLimits limits;
limits.lower = Eigen::VectorXd::Constant(6, -M_PI);
limits.upper = Eigen::VectorXd::Constant(6, M_PI);
limits.velocity_limit = Eigen::VectorXd::Constant(6, 1.0);
kinematics.setJointLimits(limits);

// 正向运动学
Eigen::VectorXd joint_positions = Eigen::VectorXd::LinSpaced(6, 0, M_PI/4);
auto end_effector_pose = kinematics.computeForwardKinematics(joint_positions);

// 逆向运动学
Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(6);
try {
    auto solution = kinematics.inverseKinematics(end_effector_pose, initial_guess);
    std::cout << "IK solution: " << solution.transpose() << std::endl;
} catch (const KinematicSolverException& e) {
    std::cerr << "IK failed: " << e.what() << std::endl;
}

// 雅可比矩阵和奇异性分析
auto jacobian = kinematics.computeJacobian(joint_positions);
auto singularity_analysis = kinematics.analyzeSingularity(joint_positions);

if (singularity_analysis.is_near_singularity) {
    std::cout << "Warning: Near singularity! Manipulability: " 
              << singularity_analysis.manipulability_index << std::endl;
}

// 速度级运动学
CartesianTwist desired_velocity;
desired_velocity.linear = Eigen::Vector3d(0.1, 0.0, 0.0);
desired_velocity.angular = Eigen::Vector3d(0.0, 0.0, 0.1);

auto joint_velocities = kinematics.inverseVelocityKinematics(
    joint_positions, desired_velocity);
```

---

## 编译和依赖

### CMakeLists.txt 配置

```cmake
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(control_pack 
    src/robot_pose_polynomial.cpp
    src/velocity_ik_generator.cpp
    src/trajectory_executor.cpp
)

target_link_libraries(control_pack
    ${orocos_kdl_LIBRARIES}
    ${KDL_PARSER_LIBRARIES}
    Eigen3::Eigen
)
```

### package.xml 依赖

```xml
<depend>orocos_kdl</depend>
<depend>kdl_parser</depend>
<depend>eigen3_cmake_module</depend>
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
```

---

## 性能考虑

### 实时安全

- `robot_pose_polynomial`：所有计算都是纯数学运算，实时安全
- `velocity_ik_generator`：预分配内存，避免运行时分配
- `trajectory_executor`：动力学计算可能较重，建议在非实时线程执行

### 内存管理

- 所有 KDL 对象都使用智能指针管理
- 预分配 Eigen 矩阵和 KDL 数组
- 避免在实时循环中动态分配内存

### 数值稳定性

- 使用阻尼最小二乘法处理奇异位形
- 雅可比矩阵条件数监控
- 关节限位检查和软约束

---

## 常见问题和解决方案

### Q: IK 求解失败怎么办？

A: 
1. 检查目标位姿是否在工作空间内
2. 尝试不同的初始猜测
3. 使用阻尼最小二乘法
4. 检查关节限位设置

### Q: 奇异位形处理？

A: 
1. 使用 `isNearSingularity()` 检测
2. 采用 `dampedLeastSquaresInverseVelocity()`
3. 监控可操作性指标
4. 规划时避开奇异区域

### Q: 轨迹插值不连续？

A: 
1. 确保轨迹点包含完整的速度和加速度信息
2. 检查时间戳是否单调递增
3. 验证边界条件设置

---

## 版本历史

- **v1.0.0** (2026-01-22): 初始版本
  - 基础运动学功能
  - 五次多项式插值
  - KDL 集成

- **v1.1.0** (2026-02-04): 架构重构
  - 分离 trajectory_executor
  - 改进文档
  - 修复拼写错误

---

## 作者和许可

**作者：** kyy (kangyangyi@hotmail.com)

**许可：** 

---

*最后更新：2026-02-04*
