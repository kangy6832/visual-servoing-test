// 计算基础关节速度
auto joint_velocities = calculate_joint_velocity(end_effector_velocity);

// 动力学补偿增量
if (kdl_dynamics_ && dynamics_params_.enable_dynamics_compensation) {
    Eigen::VectorXd gravity = kdl_dynamics_->calculateGravityCompensation(current_joint_positions);
    Eigen::VectorXd coriolis = kdl_dynamics_->calculateCoriolisCompensation(current_joint_positions, current_joint_velocities);

    // 可选：使用期望末端加速度计算惯性补偿
    Eigen::VectorXd joint_accel = kdl_dynamics_->calculateJointAcceleration(
        current_joint_positions, current_joint_velocities, ee_acceleration, jacobian
    );

    Eigen::VectorXd inertia_comp = kdl_dynamics_->calculateDynamicsTorque(
        current_joint_positions, current_joint_velocities, joint_accel
    );

    // 总动力学补偿力矩
    Eigen::VectorXd tau_ff = gravity + coriolis + inertia_comp;

    // 转换成关节速度修正量（最简单方式：增量调速）
    double gain = dynamics_params_.compensation_gain; // 小于1
    for (int i = 0; i < joint_velocities.size(); ++i) {
        // M^-1 * tau_ff ≈ K * tau_ff (近似)
        joint_velocities(i) += gain * tau_ff(i);
    }
}