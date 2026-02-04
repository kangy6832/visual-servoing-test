

#include "control_pack/robot_pose_polynomial.hpp"
#include <chrono>



namespace RobotPosePolynomial{





    /**
     * @brief 设置五次多项式轨迹参数
     *
     * @details
     * 根据给定的起止时间、起止位置、速度、加速度，
     * 计算满足以下边界条件的五次多项式：
     *
     *  p(t0) = p0        起始位置
     *  p'(t0) = v0       起始速度
     *  p''(t0) = a0      起始加速度
     *
     *  p(t1) = pt        目标位置
     *  p'(t1) = v1       目标速度
     *  p''(t1) = at      目标加速度
     *
     * 多项式形式：
     *  p(t) = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f
     *
     * 该函数只负责计算系数，不负责轨迹采样。
     *
     * @param[in] t0 起始时间
     * @param[in] t1 终止时间
     * @param[in] p0 起始位置
     * @param[in] v0 起始速度
     * @param[in] a0 起始加速度
     * @param[in] pt 目标位置
     * @param[in] v1 目标速度
     * @param[in] at 目标加速度
     */
    void QuinticParam::set_param(
        const double t0, const double t1, const double p0, const double v0, 
        const double a0, const double pt, const double v1, const double at
    ){
        /* 轨迹总时常 */
        double T = t1 - t0;

        /* 各阶段时间项 */
        double T2 = T * T;      // 二阶
        double T3 = T2 * T;     // 三阶
        double T4 = T3 * T;
        double T5 = T4 * T;

       /**
         * 低阶项直接由起始状态确定
         *
         * p(t0) = f
         * p'(t0) = e
         * p''(t0) = 2d
         */
        f = p0;        // 常数项：初始位置
        e = v0;        // 一次项：初始速度
        d = a0 / 2.0;  // 二次项：初始加速度 / 2

        /**
         * 高阶项（a, b, c）由终止约束求解
         *
         * 通过代入 t = t1 的位置、速度、加速度条件，
         * 解线性方程组得到以下解析解。
         *
         * 这些公式是标准五次多项式轨迹生成结果。
         */
        a = (12 * (pt - p0) - 6 * (v1 + v0) * T - (at - a0) * T2) / (2 * T5);
        b = (-30 * (pt - p0) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * at) * T2) / (2 * T4);
        c = (20 * (pt - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - at) * T2) / (2 * T3);

        /* 记录轨迹的起止时间 */
        this->t0 = t0;
        this->t1 = t1;
    }





    /**
     * @brief 计算五次多项式在时间 t 的位置值
     *
     * @details
     * 根据当前时间 t，在对应的轨迹段内计算位置值。
     * 五次多项式以 (t0, t1) 为定义区间：
     *
     *  p(t) = a*tau^5 + b*tau^4 + c*tau^3 + d*tau^2 + e*tau + f
     *  其中 tau = t - t0
     *
     * 对于区间外的时间：
     * - t <= t0：返回起始位置（防止数值外推）
     * - t >= t1：返回终点位置（保证轨迹不越界）
     *
     * 这种处理方式可以：
     * - 避免轨迹执行超时导致的位置发散
     * - 提供数值和控制上的安全性
     *
     * @param[in] t 当前时间（通常为相对轨迹时间）
     * @return 对应时间的期望位置值
     */
    double QuinticParam::get_position(const double t){
        /* 时间早于轨迹起点，直接返回起始位置 */
        if(t <= t0)
            return f;
        
        /* 时间晚于轨迹终点，返回终点位置 */
        if(t >= t1){
            const double T = t1 - t0;

            /* p(t1) = p(T) ，使用完整时间长度计算 */
            return (
                a*T*T*T*T*T + 
                b*T*T*T*T + 
                c*T*T*T + 
                d*T*T + 
                e*T + 
                f
            );
        }

        /* 正常轨迹区间：使用相对时间 tau 计算 */
        const double tau = t - t0;

        return (
            a * tau * tau * tau * tau * tau + 
            b * tau * tau * tau * tau + 
            c * tau * tau * tau + 
            d * tau * tau + 
            e * tau + 
            f
        );
    }





    /**
     * @brief 计算五次多项式在时间 t 的速度值
     *
     * @details
     * 速度为位置五次多项式对时间的一阶导数：
     *
     *  p(tau) = a*tau^5 + b*tau^4 + c*tau^3 + d*tau^2 + e*tau + f
     *
     *  v(tau) = dp/dt
     *         = 5*a*tau^4 + 4*b*tau^3 + 3*c*tau^2 + 2*d*tau + e
     *
     *  其中 tau = t - t0
     *
     * 与位置计算一致：
     * - t <= t0 ：返回起始速度
     * - t >= t1 ：返回终止速度
     *
     * 这种“时间夹紧”策略可以防止轨迹执行超时导致的速度外推问题。
     *
     * @param[in] t 当前时间（相对轨迹时间）
     * @return 对应时间的期望速度值
     */
    double QuinticParam::get_velocity(const double t){
        /* 时间早于轨迹起点，返回起始速度 */
        if(t <= t0)
            return e;

        /* 时间超过轨迹终点，返回终止速度 */
        if(t >= t1){
            const double T = t1 - t0;

            /* v(t1) = v(T)，使用完整时间长度计算 */
            return (
                5*a*T*T*T*T + 
                4*b*T*T*T +
                3*c*T*T +
                2*d*T +
                e
            );
        }
        
        /* 正常轨迹区间：使用相对时间 tau 计算 */
        const double tau = t - t0;

        return (
            5 * a * tau * tau * tau * tau + 
            4 * b * tau * tau * tau + 
            3 * c * tau * tau + 
            2 * d * tau + 
            e
        );
    }





    /**
     * @brief 计算五次多项式在时间 t 的加速度值
     *
     * @details
     * 加速度为位置五次多项式对时间的二阶导数：
     *
     *  p(tau) = a*tau^5 + b*tau^4 + c*tau^3 + d*tau^2 + e*tau + f
     *
     *  v(tau) = dp/dt
     *         = 5*a*tau^4 + 4*b*tau^3 + 3*c*tau^2 + 2*d*tau + e
     *
     *  a(tau) = d²p/dt²
     *         = 20*a*tau^3 + 12*b*tau^2 + 6*c*tau + 2*d
     *
     *  其中 tau = t - t0
     *
     * 与位置、速度计算方式一致：
     * - t <= t0 ：返回起始加速度
     * - t >= t1 ：返回终止加速度
     *
     * 通过时间区间夹紧，保证加速度在轨迹边界处不发生突变，
     * 从而维持整体轨迹的 C² 连续性。
     *
     * @param[in] t 当前时间（相对轨迹时间）
     * @return 对应时间的期望加速度值
     */
    double QuinticParam::get_acceleration(const double t){
        /* 时间早于轨迹起点，返回起始加速度 */
        if(t <= t0)
            return 2.0 * d; // 因为 d = a0 / 2

        /* 时间超过轨迹终点，返回终止加速度 */
        if(t >= t1){
            const double T = t1 - t0;

            /* a(t1) = a(T)，使用完整时间长度计算 */
            return (
                20 * a * T * T * T + 
                12 * b * T * T + 
                6 * c * T + 
                2 * d
            );
        }

        /* 正常轨迹区间：使用相对时间 tau 计算 */
        const double tau = t - t0;

        return (
            20 * a * tau * tau * tau + 
            12 * b * tau * tau + 
            6 * c * tau + 
            2 * d
        );
    }




    /**
     * @brief 连续轨迹管理器构造函数
     *
     * @details
     * 初始化内部状态，将当前轨迹段索引重置为 0。
     * 表示轨迹尚未开始执行，后续将从第一段
     * （trajectory.points[0] -> trajectory.points[1]）开始。
     */
    ContinuousTrajectory::ContinuousTrajectory(){ cur_index = 0; }




    /**
     * @brief 初始化指定轨迹段的五次多项式参数
     *
     * @details
     * 使用 trajectory.points[index] 和 trajectory.points[index + 1]，
     * 为每个关节生成一段五次多项式轨迹。
     *
     * 每一段轨迹保证：
     * - 起止位置、速度、加速度连续
     * - 段内平滑（C² 连续）
     *
     * @param index 当前轨迹段起点索引
     *              实际使用的区间是 [index, index + 1]
     */
    void ContinuousTrajectory::init_segment(std::size_t index){

        /* 越界保护：必须保证 index 和 index+1 都存在 */
        if(index + 1 >= trajectory.points.size())
            return;
        
        /* 轨迹段起点和终点 */
        const auto& P0 = trajectory.points[index];
        const auto& P1 = trajectory.points[index + 1];

        /* 起止时间（统一转换为秒） */
        const double t0 = P0.time_from_start.sec +
                          P0.time_from_start.nanosec * 1e-9;
                    
        const double t1 = P1.time_from_start.sec + 
                          P1.time_from_start.nanosec * 1e-9;

        /**
         * 为每个关节初始化一条五次多项式
         *
         * line[i] 表示第 i 个关节在当前轨迹段的插值器
         */
        for (int i = 0 ; i < 6 ; ++i){
            line[i].set_param(
                t0, t1, 
                P0.positions[i],
                P0.velocities[i],
                P0.accelerations[i],
                P1.positions[i],
                P1.velocities[i],
                P1.accelerations[i]
            );
        }
                          
    }




    /**
     * @brief 获取当前时间对应的期望关节轨迹点
     *
     * @details
     * 根据当前时间 time：
     * 1. 计算相对轨迹起始时间 t
     * 2. 判断是否需要切换轨迹段
     * 3. 使用当前段的五次多项式计算位置、速度、加速度
     *
     * @param[in]  time   当前时间（系统时间）
     * @param[out] output 输出的关节期望状态（pos / vel / acc）
     * @return true  成功计算
     * @return false 轨迹已结束或轨迹非法
     */
    bool ContinuousTrajectory::get_target(
        const rclcpp::Time& time, trajectory_msgs::msg::JointTrajectoryPoint& output){

        /* 基本保护, 轨迹至少需要两个点才能插值 */
        if(trajectory.points.size() < 2)
            return false;
        
        /* 当前轨迹相对时间 */
        const double t = (time - start_time).seconds();

        /**
         * 第一次进入时，显式初始化第一段轨迹
         * 防止第一段未初始化就被使用
         */
        if(!initialized_){
                init_segment(0);
                initialized_ = true;
            }


        /**
         * 轨迹段切换逻辑：
         * 如果当前时间已经超过下一轨迹点的 time_from_start，
         * 则进入下一段轨迹，并重新初始化五次多项式参数
         *
         * while 用于应对：
         * - 控制周期过慢
         * - time 一次跳过多个轨迹点
         */
        while(
            cur_index + 1 < trajectory.points.size() && 
            t >= trajectory.points[cur_index + 1].time_from_start.sec + 
                 trajectory.points[cur_index + 1].time_from_start.nanosec * 1e-9
        ){
            cur_index++;
            init_segment(cur_index);
        }

        /* 已经执行到最后一个轨迹点，轨迹结束 */
        if(cur_index + 1 >= trajectory.points.size())
            return false;

        /* 为输出消息分配空间 */
        output.positions.resize(6);
        output.velocities.resize(6);
        output.accelerations.resize(6);
        
        /**
         * 使用当前轨迹段的五次多项式，
         * 计算在时间 t 的位置、速度和加速度
         */
        for(int i = 0 ; i < 6 ; ++i){
            output.positions[i] = line[i].get_position(t);
            output.velocities[i] = line[i].get_velocity(t);
            output.accelerations[i] = line[i].get_acceleration(t);
        }

        return true;
    }





    /**
     * @brief 启动轨迹跟踪
     *
     * @details
     * 设置轨迹的起始时间，并重置内部状态，
     * 用于开始一次新的轨迹执行。
     *
     * @param now 当前系统时间，作为轨迹 time_from_start 的零点
     */
    void ContinuousTrajectory::start_track(rclcpp::Time now){
        this->start_time = std::move(now);
        cur_index = 0;
        initialized_ = false; // 必须清空，避免复用上一次轨迹段
    }





    /**
     * @brief 设置要执行的关节轨迹
     *
     * @details
     * 保存 FollowJointTrajectory 生成的轨迹，
     * 实际插值和时间推进由 get_target 负责。
     *
     * @param trajectory 输入的关节轨迹
     */
    void ContinuousTrajectory::set_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory){
        this->trajectory = trajectory;
    }





} // namespace RobotPosePolynomial