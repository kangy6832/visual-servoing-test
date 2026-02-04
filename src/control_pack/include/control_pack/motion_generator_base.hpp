#ifndef MOTION_GENERATOR_BASE_HPP
#define MOTION_GENERATOR_BASE_HPP
/**
 * motion_generator_base.hpp
 *
 * 统一的 Motion Generator 接口（位置/速度/加速度输出）。
 *
 * 设计要点:
 *  - MotionGenerator 负责产生 DesiredState（三个向量：q, dq, ddq）。
 *  - update(...) 必须实时安全：不分配内存、不做阻塞 I/O。
 *  - reset(...) 在模式切换时调用，用于把 generator 状态初始化为当前机器人实际状态（避免跳变）。
 *
 * 使用方法（伪）:
 *   RobotState cur = read_state_from_hardware();
 *   DesiredState desired;
 *   ControlMode mode;
 *   generator->update(cur, now, desired, mode);
 *
 * 注意:
 *  - 推荐在 controller 激活阶段为 DesiredState 向量提前 resize 到关节数以避免运行时分配。
 *  - 如果实现里需要临时大矩阵，请复用成员缓冲区而不是在 update 中分配。
 */

#include <Eigen/Dense>
#include <rclcpp/time.hpp>
#include <vector>
namespace motion_generator{
    /**
     * @brief 当前机器人关节状态（接受硬件反馈 /  state interface）
     */
    struct RobotState {
        Eigen::VectorXd q;    // current positions(size = N)
        Eigen::VectorXd dq;   // current velocities(size = N)

        RobotState() = default;
        RobotState(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_) : 
            q(q_), dq(dq_) {}
    };

    /**
     * @brief 期望轨迹点（统一输出）
     *
     * 注意：为了 realtime safety，建议外部在启动时 allocate 好内存并维持大小不变。
     */
    struct DesiredState{
        Eigen::VectorXd q;     // desired positions
        Eigen::VectorXd dq;    // desired velocities
        Eigen::VectorXd ddq;   // desired accelerations

        DesiredState() = default;
    }

    /**
     * @brief 控制模式（由 generator 指定，controller 依据此决定使用 positions/velocity 接口）
     */
    enum class ControlMode{
        POSITION, // 使用 position 跟踪（q -> position controller)
        VELOCITY  // 使用 velocity 跟踪（dq -> velocity controller）
    };

    /**
     * @class MotionGeneratorBase
     * @brief Motion Generator 抽象基类
     *
     * 每个派生类都需要实现:
     *  - reset(...) : 在切换到该 generator 时调用
     *  - update(...) : 每控制周期调用，填充 DesiredState，告知 ControlMode
     *
     * 语义:
     *  - reset(...) 必须保证将内部状态对齐到当前机器人状态（用于无冲击切换）。
     *  - update(...) 返回 true 表示产生有效输出（controller 可使用）；返回 false 表示当前 generator 暂不可用（controller 应进行安全处理）。
     */
    class MotionGeneratorBase{
        public:
            MotionGeneratorBase() = default;
            virtual ~MotionGeneratorBase() = default;

            // 禁用拷贝
            MotionGeneratorBase(const MotionGeneratorBase&) = delete;
            MotionGeneratorBase& operator=(const MotionGeneratorBase&) = delete;

            /**
             * @brief reset internal state (called on mode switch)
             * @param current 最新机器人状态（由 controller 提供）
             * @param now 当前时间（rclcpp::Clock::now()）
             *
             * 注意：reset 可以分配资源（非实时），但应尽量轻量。
             */
            virtual void reset(const RobotState& current, const rclcpp::Time& now) = 0;

            /**
             * @brief compute desired state at current time
             * @param current 当前机器人状态（实时安全数据）
             * @param now 当前时间戳
             * @param desired 输出：期望状态（提前 resize 好以避免分配）
             * @param mode 输出：本次期望的控制模式（POSITION / VELOCITY）
             * @return true 表示输出有效，可被 controller 使用；false 表示不可用（例如正在初始化）
             *
             * 实时安全要求：
             *  - 不在此函数中进行 new/delete / 文件 I/O / 等待操作
             *  - 如果需要缓存中间结果，请在类内部预分配 buffer
             */
            virtual bool update(
                const RobotState& current ,
                const rclcpp::Time& now, 
                DesiredState& desired, 
                ControlMode& mode) = 0;
    };

} // namespace motion_generator

#endif // !MOTION_GENERATOR_BASE_HPP