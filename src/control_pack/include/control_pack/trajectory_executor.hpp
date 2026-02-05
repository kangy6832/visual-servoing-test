/**
 * @file trajectory_executor.hpp
 * @brief 轨迹执行器模块 - 负责机器人轨迹执行、动力学计算和ROS2 Action服务
 * 
 * @details 本模块将原本混合在 robot_pose_polynomial 中的以下功能分离出来：
 *          - KDL 运动学和动力学计算
 *          - ROS2 Action 服务器处理  
 *          - 轨迹执行状态管理
 * 
 * 设计目标：
 * 1. 职责分离：将轨迹插值算法与执行逻辑解耦
 * 2. 模块化：动力学计算可独立复用
 * 3. 实时安全：避免在控制循环中重计算
 * 
 * 主要功能：
 * - 从URDF初始化KDL运动学链和动力学求解器
 * - 提供关节空间动力学计算接口（τ = M·q̈ + C·q̇ + G）
 * - 处理FollowJointTrajectory Action的回调（目标接收/取消/执行）
 * - 管理轨迹执行状态（执行中/取消/完成）
 * 
 * 使用场景：
 * - 机器人轨迹跟踪控制
 * - 关节空间动力学补偿
 * - ROS2 Action服务器实现
 * 
 * @author kyy (kangyangyi@hotmail.com)
 * @date 2026-02-04
 * @version 1.1.0
 */

#pragma once

#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/logging.hpp>

namespace trajectory_executor {
    /**
     * @class TrajectoryExecutor
     * @brief 轨迹执行器 - 整合KDL动力学、运动学和ROS2 Action服务
     * 
     * @details 本类是轨迹执行的核心组件，想解决以下问题：
     *          1. 职责混乱：原本动力学计算与轨迹插值混合在一起
     *          2. 复用困难：KDL功能无法独立使用
     *          3. 维护复杂：Action回调与算法逻辑耦合
     * 
     * 设计思路：
     * - 单一职责：专门负责轨迹执行相关功能
     * - 接口清晰：提供明确的初始化和计算接口
     * - 状态管理：统一管理执行状态，便于外部查询
     * 
     * 核心职责：
     * 1. KDL初始化：从URDF解析机器人模型，建立运动学链
     * 2. 动力学计算：提供关节空间动力学方程求解
     * 3. Action服务：处理轨迹执行的标准ROS2 Action
     * 4. 状态跟踪：记录轨迹执行的生命周期状态
     * 
     * 使用示例：
     * @code
     * // 创建执行器并初始化
     * auto executor = std::make_shared<TrajectoryExecutor>(node);
     * std::string urdf = node->get_parameter("robot_description").as_string();
     * if (!executor->initFromURDF(urdf)) {
     *     RCLCPP_ERROR(node->get_logger(), "初始化失败");
     *     return;
     * }
     * 
     * // 设置关节状态并计算动力学
     * executor->q_kdl.data = current_positions;
     * executor->dq_kdl.data = current_velocities; 
     * executor->ddq_kdl.data = desired_accelerations;
     * auto torques = executor->dynamicCalc();
     * @endcode
     * 
     * @note 本类专注于执行层，不包含轨迹规划算法
     * @warning 动力学计算较重，建议在非实时线程中调用
     */
    class TrajectoryExecutor {
        public:
            /**
             * @brief 构造函数 - 初始化轨迹执行器的基本组件
             * 
             * @details 构造函数想做的事情：
             *          1. 保存ROS2节点引用，用于日志记录和参数访问
             *          2. 初始化重力向量（默认-9.81 m/s²）
             *          3. 创建参数客户端（可选，用于获取robot_description）
             *          4. 将所有KDL对象初始化为空指针
             * 
             * 设计考虑：
             * - 延迟初始化：KDL对象在initFromURDF()中创建
             * - 资源安全：使用智能指针管理内存
             * - 异常安全：构造函数不抛出异常
             * 
             * @param node ROS2节点共享指针，用于：
             *              - 日志输出（RCLCPP_INFO/WARN/ERROR）
             *              - 参数访问（get_parameter）
             *              - 时钟获取（now()）
             *              - 创建Action服务器
             * 
             * @note 构造后必须调用initFromURDF()才能使用动力学功能
             * @warning 不要在构造函数中进行耗时操作
             */
            explicit TrajectoryExecutor(const rclcpp::Node::SharedPtr node);
            ~TrajectoryExecutor() = default;

            // 禁止拷贝
            TrajectoryExecutor(const TrajectoryExecutor&) = delete;
            TrajectoryExecutor& operator=(const TrajectoryExecutor&) = delete;

            /**
             * @brief 从URDF初始化KDL运动学链和动力学求解器
             * 
             * @details 本方法想解决的核心问题：
             *          1. 模型解析：将URDF转换为KDL可用的运动学表示
             *          2. 链提取：从完整机器人模型中提取执行链
             *          3. 求解器创建：建立动力学计算所需的KDL对象
             *          4. 内存预分配：避免运行时动态分配
             * 
             * 执行流程：
             * Step 1: 调用kdl_parser::treeFromString()解析URDF
             * Step 2: 调用tree.getChain()提取base_link到tool0链
             * Step 3: 创建ChainDynParam求解器（考虑重力）
             * Step 4: 预分配所有KDL数组到关节数量
             * 
             * 设计考虑：
             * - 错误处理：每步都有失败检查和日志输出
             * - 资源管理：使用智能指针自动管理KDL对象
             * - 性能优化：预分配数组，避免实时循环中分配
             * 
             * @param urdf_xml URDF字符串描述，包含：
             *                  - 机器人连杆和关节定义
             *                  - 坐标系关系
             *                  - 物理参数（质量、惯量）
             * 
             * @return true 初始化成功，所有KDL对象可用
             * @return false 初始化失败，检查日志获取错误信息
             * 
             * @return 初始化状态：
             *          - true: tree, chain, dyn都有效
             *          - false: 至少一个组件初始化失败
             * 
             * @note 必须在使用dynamicCalc()之前调用此方法
             * @note URDF必须包含base_link和tool0坐标系
             * @warning 此方法可能较耗时，建议在初始化阶段调用
             * @see dynamicCalc() 依赖此初始化
             * @see kdl_parser::treeFromString() 底层解析函数
             */
            bool initFromURDF(const std::string& urdf_xml);

            /**
             * @brief 计算关节空间动力学 - 基于牛顿-欧拉方程的关节力矩求解
             * 
             * @details 本方法想实现的动力学模型：
             *          τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q)
             * 
             * 物理意义：
             * 1. M(q)·q̈: 惯性力项 - 加速关节所需的力矩
             * 2. C(q,q̇)·q̇: 科里奥利和离心力项 - 速度耦合产生的力矩
             * 3. G(q): 重力项 - 克服重力所需的力矩
             * 
             * 计算流程：
             * Step 1: 调用KDL的JntToMass()计算惯性矩阵M
             * Step 2: 调用KDL的JntToCoriolis()计算科里奥利项C
             * Step 3: 调用KDL的JntToGravity()计算重力项G
             * Step 4: 组合计算总关节力矩τ
             * 
             * 应用场景：
             * - 动力学补偿控制：前馈补偿重力、惯性等
             * - 力矩限制检查：验证控制输出在安全范围内
             * - 仿真分析：分析机器人动力学特性
             * 
             * 前置条件：
             * - 必须先调用initFromURDF()成功初始化
             * - 需要设置q_kdl, dq_kdl, ddq_kdl的当前值
             * 
             * @return 6维关节力矩向量 [N·m]，每个元素对应一个关节
             * @return 如果未初始化，返回零向量并输出警告
             * 
             * @note 计算结果基于当前设置的关节状态
             * @note 重力向量默认为(0,0,-9.81)，可在初始化后修改
             * @warning 动力学计算相对较重，不建议在高频控制循环中直接调用
             * @warning 确保关节状态的单位正确（位置rad，速度rad/s，加速度rad/s²）
             * 
             * @see initFromURDF() 必须先调用此方法
             * @see q_kdl 关节位置数组（需要预先设置）
             * @see dq_kdl 关节速度数组（需要预先设置）
             * @see ddq_kdl 关节加速度数组（需要预先设置）
             * @see KDL::ChainDynParam 底层动力学求解器
             */
            Eigen::Vector<double, 6> dynamicCalc();

            // ============================================================
            // Action 服务器回调 - 处理ROS2标准轨迹执行Action
            // ============================================================

            /**
             * @brief 处理新的轨迹执行目标 - FollowJointTrajectory Action的目标接收回调
             * 
             * @details 本回调想解决的问题：
             *          1. 目标验证：检查轨迹是否可执行
             *          2. 资源检查：确认机器人状态允许执行
             *          3. 响应生成：返回接受或拒绝的决定
             * 
             * 调用时机：
             * - 客户端发送新的FollowJointTrajectory Goal时
             * - Action服务器自动调用此方法进行目标验证
             * 
             * 设计考虑：
             * - 快速响应：避免阻塞客户端过久
             * - 安全检查：防止执行危险轨迹
             * - 日志记录：便于调试和监控
             * 
             * @param uuid 目标唯一标识符，用于：
             *              - 日志关联
             *              - 取消操作定位
             *              - 状态跟踪
             * 
             * @param goal FollowJointTrajectory目标指针，包含：
             *              - trajectory: 要执行的关节轨迹
             *              - goal_time_tolerance: 时间容差
             *              - goal_tolerance: 位置容差
             * 
             * @return GoalResponse 接受决策：
             *          - ACCEPT_AND_EXECUTE: 接受并开始执行
             *          - REJECT: 拒绝执行（原因见日志）
             * 
             * @note 当前实现总是接受，实际应用应添加验证逻辑
             * @todo 添加轨迹验证：时间约束、关节限制、速度限制
             * @todo 添加冲突检查：是否与当前执行冲突
             * @see handle_accepted() 目标接受后的处理
             * @see control_msgs::action::FollowJointTrajectory 标准Action定义
             */
            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID& uuid, 
                const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal
            );

            /**
             * @brief 处理轨迹取消请求 - FollowJointTrajectory Action的取消回调
             * 
             * @details 本回调想实现的取消策略：
             *          1. 优雅停止：平滑减速到停止
             *          2. 资源释放：清理执行线程和状态
             *          3. 状态同步：更新执行状态标志
             * 
             * 调用时机：
             * - 客户端发送取消请求时
             * - Action服务器自动调用此方法
             * 
             * 取消场景：
             * - 用户手动停止轨迹
             * - 安全触发（急停、碰撞检测）
             * - 新目标到达，替换当前执行
             * 
             * @param goal_handle 要取消的目标句柄，提供：
             *              - 目标状态查询
             *              - 结果发布接口
             *              - 取消确认机制
             * 
             * @return CancelResponse 取消响应：
             *          - ACCEPT: 接受取消请求
             *          - REJECT: 拒绝取消（已完成或不存在）
             * 
             * @note 当前实现总是接受取消
             * @todo 实现优雅的轨迹停止逻辑
             * @todo 添加取消原因验证
             * @see handle_goal() 目标接受回调
             * @see is_execut_trajectory 执行状态标志
             */
            rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
            );

            /**
             * @brief 处理已接受的轨迹目标 - 启动轨迹执行的主要入口
             * 
             * @details 本方法想实现的执行策略：
             *          1. 线程管理：创建独立的执行线程
             *          2. 状态设置：更新执行状态标志
             *          3. 资源准备：初始化执行所需资源
             * 
             * 调用时机：
             * - handle_goal()返回ACCEPT_AND_EXECUTE后
             * - Action服务器自动调用此方法
             * 
             * 执行流程（待实现）：
             * Step 1: 解析轨迹消息，提取关键点
             * Step 2: 初始化轨迹插值器
             * Step 3: 启动实时控制循环
             * Step 4: 监控执行状态，处理完成/取消
             * 
             * 设计考虑：
             * - 非阻塞：立即返回，不阻塞Action服务器
             * - 异常安全：执行线程异常不影响主线程
             * - 状态一致：确保状态标志正确反映执行状态
             * 
             * @param goal_handle 目标句柄，用于：
             *              - 发布执行结果（成功/失败/中止）
             *              - 检查取消状态
             *              - 更新执行进度
             * 
             * @note 当前实现为空，需要添加实际执行逻辑
             * @todo 实现轨迹解析和插值器初始化
             * @todo 创建执行线程和控制循环
             * @todo 添加执行进度反馈机制
             * @todo 实现异常处理和资源清理
             * @see robot_pose_polynomial::ContinuousTrajectory 轨迹插值器
             * @see dynamicCalc() 动力学计算接口
             */
            void handle_accepted(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle
            );

            // ============================================================
// 执行状态管理 - 轨迹执行生命周期的状态标志
// ============================================================

            /** @brief 是否正在执行轨迹 - 标识当前是否有活跃的轨迹执行任务
             * 
             * 用途：
             * - 外部查询执行状态
             * - 防止重复启动执行
             * - 控制循环中的状态判断
             * 
             * 状态转换：
             * false → true: 调用handle_accepted()后设置
             * true → false: 轨迹完成或取消后设置
             * 
             * @note 此标志由Action回调管理，外部只读
             * @see handle_accepted() 设置此标志为true
             * @see finished_execut 轨迹完成标志
             */
            bool is_execut_trajectory{false};
            
            /** @brief 是否请求取消执行 - 标识是否收到轨迹取消请求
             * 
             * 用途：
             * - 信号通知执行线程停止
             * - 优雅停止而非强制中断
             * - 取消状态的外部查询
             * 
             * 设置时机：
             * - handle_cancel()接受取消请求后
             * - 外部调用停止接口时
             * 
             * @note 执行线程应定期检查此标志
             * @see handle_cancel() 取消请求处理
             */
            bool cancle_execut{false};
            
            /** @brief 轨迹执行是否已完成 - 标识当前轨迹是否正常结束
             * 
             * 用途：
             * - 标识轨迹执行的成功完成
             * - 触发结果发布和资源清理
             * - 外部查询执行结果
             * 
             * 设置时机：
             * - 轨迹所有点执行完毕
             * - 到达轨迹终点时间
             * 
             * @note 与取消状态互斥，不能同时为true
             * @see is_execut_trajectory 执行状态标志
             */
            bool finished_execut{false};

        private:
            // ============================================================
            // KDL 与动力学组件 - 底层计算引擎
            // ============================================================
            
            /** @brief KDL运动学树 - 从URDF解析的完整机器人运动学结构
             * 
             * 用途：
             * - 存储完整的机器人运动学关系
             * - 提取特定的运动学链
             * - 支持多链机器人配置
             * 
             * 初始化：
             * - initFromURDF()中从URDF字符串解析
             * - 包含所有连杆和关节的运动学关系
             * 
             * @note 主要用于链提取，不直接参与计算
             * @see chain 从此树提取的执行链
             * @see initFromURDF() 初始化方法
             */
            KDL::Tree tree;
            
            /** @brief KDL运动学链 - 从base_link到tool0的执行链
             * 
             * 用途：
             * - 正向运动学计算（关节→末端位姿）
             * - 雅可比矩阵计算（关节速度→末端速度）
             * - 动力学参数求解器的输入
             * 
             * 链定义：
             * - 起始关节：base_link（机器人基座）
             * - 终止关节：tool0（末端工具）
             * - 包含中间所有关节的运动学关系
             * 
             * @note 这是实际计算中使用的主要对象
             * @see dyn 基于此链的动力学求解器
             * @see computeForwardKinematics() 正向运动学计算
             */
            KDL::Chain chain;
            
            /** @brief URDF字符串缓存 - 机器人模型的XML描述
             * 
             * 用途：
             * - 调试和日志记录
             * - 重新初始化时的快速加载
             * - 模型验证和检查
             * 
             * 内容：
             * - 机器人连杆定义（link）
             * - 关节参数（joint）
             * - 物理属性（质量、惯量）
             * - 坐标系变换关系
             * 
             * @note 通常从robot_description参数获取
             * @see initFromURDF() 使用此字符串初始化
             */
            std::string urdf_xml;
            
            /** @brief ROS2节点指针 - 用于日志、参数和时钟的节点引用
             * 
             * 用途：
             * - 日志输出（RCLCPP_INFO/WARN/ERROR）
             * - 参数访问（get_parameter）
             * - 时间获取（now()）
             * - 创建Action服务器和客户端
             * 
             * 生命周期：
             * - 构造函数中设置
             * - 整个对象生命周期有效
             * - 不手动管理，由外部保证有效性
             * 
             * @note 所有ROS2相关操作都依赖此指针
             * @warning 确保节点在对象使用期间保持有效
             */
            rclcpp::Node::SharedPtr node;
            
            /** @brief 机器人描述参数客户端 - 用于获取robot_description参数
             * 
             * 用途：
             * - 从参数服务器获取URDF描述
             * - 支持参数动态更新
             * - 提供参数访问的异步接口
             * 
             * 设计考虑：
             * - 异步获取：避免阻塞初始化
             * - 容错处理：参数不存在时的降级策略
             * - 缓存机制：减少参数服务器访问
             * 
             * @note 当前实现为SyncParametersClient，可改为AsyncParametersClient
             * @see urdf_xml 存储获取的URDF字符串
             */
            rclcpp::SyncParametersClient::SharedPtr robot_description_param_;

            /** @brief 重力向量 - 动力学计算中的重力加速度向量
             * 
             * 用途：
             * - 动力学方程中的重力项计算
             * - KDL ChainDynParam的重力配置
             * - 支持不同重力环境（地面、空间等）
             * 
             * 默认值：
             * - (0.0, 0.0, -9.81) m/s²
             * - 对应地球重力，Z轴向下
             * 
             * 配置：
             * - 构造函数中初始化为默认值
             * - 可在初始化后修改以适应不同环境
             * 
             * @note 单位为m/s²，方向与坐标系一致
             * @see dyn 使用此重力的动力学求解器
             */
            KDL::Vector gravity;
            
            /** @brief KDL动力学参数求解器 - 关节空间动力学计算的核心引擎
             * 
             * 用途：
             * - 计算惯性矩阵M(q)
             * - 计算科里奥利项C(q,q̇)
             * - 计算重力项G(q)
             * 
             * 功能：
             * - JntToMass(): 惯性矩阵计算
             * - JntToCoriolis(): 科里奥利力计算
             * - JntToGravity(): 重力补偿计算
             * 
             * 依赖：
             * - chain: 运动学链定义
             * - gravity: 重力向量配置
             * 
             * @note 所有动力学计算都通过此对象进行
             * @see dynamicCalc() 使用此求解器的主要接口
             * @see chain 求解器依赖的运动学链
             */
            std::shared_ptr<KDL::ChainDynParam> dyn;
            
            /** @brief 关节空间惯性矩阵 - 存储当前关节构型下的惯性矩阵M(q)
             * 
             * 用途：
             * - 缓存dynamicCalc()的计算结果
             * - 避免重复计算提高性能
             * - 提供惯性矩阵的外部访问
             * 
             * 属性：
             * - 维度：n×n（n为关节数量）
             * - 单位：kg·m²
             * - 对称正定矩阵
             * 
             * 更新：
             * - 每次dynamicCalc()调用时更新
             * - 反映当前关节构型的惯性特性
             * 
             * @note 主要用于动力学分析和控制算法
             * @see dynamicCalc() 更新此矩阵的方法
             */
            KDL::JntSpaceInertiaMatrix M_kdl;
            
            /** @brief 科里奥利力和离心力向量 - 存储C(q,q̇)·q̇项的计算结果
             * 
             * 用途：
             * - 缓存速度相关动力学项
             * - 与关节速度相乘得到科里奥利力矩
             * - 提供动力学分析的数据接口
             * 
             * 物理意义：
             * - 科里奥利力：关节速度耦合产生的惯性力
             * - 离心力：关节旋转产生的离心力
             * 
             * 属性：
             * - 维度：n×1（n为关节数量）
             * - 单位：N·m
             * - 依赖于关节位置和速度
             * 
             * @note 在高速运动时影响显著
             * @see dynamicCalc() 更新此向量的方法
             */
            KDL::JntArray C_kdl, G_kdl;
            
            /** @brief 关节状态数组组 - 存储当前关节的运动学状态
             * 
             * q_kdl: 关节位置 [rad] - 当前关节角度
             * dq_kdl: 关节速度 [rad/s] - 当前关节角速度  
             * ddq_kdl: 关节加速度 [rad/s²] - 当前关节角加速度
             * 
             * 用途：
             * - dynamicCalc()的输入参数
             * - 运动学计算的当前状态
             * - 控制算法的状态反馈
             * 
             * 设置方式：
             * - 外部直接赋值（从硬件或状态估计获取）
             * - 在每次控制循环开始前更新
             * - 确保与实际机器人状态同步
             * 
             * @note 必须在调用dynamicCalc()前正确设置
             * @warning 确保单位正确，避免计算错误
             * @see dynamicCalc() 使用这些数组的动力学计算
             */
            KDL::JntArray q_kdl, dq_kdl, ddq_kdl;
    };
} // namespace trajectory_executor
