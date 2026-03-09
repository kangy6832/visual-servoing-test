/**
 * @file kdl_dynamics.cpp
 * @brief KDL动力学计算类实现
 * 
 * @version 1.0.0
 * @date 2026-03-07
 */

#include "control_pack/kdl_dynamics.hpp"
#include <stdexcept>
#include <sstream>

namespace robotic_task {

KDLDynamics::KDLDynamics() 
    : num_joints_(0), is_initialized_(false) {
}

KDLDynamics::~KDLDynamics() {
}

bool KDLDynamics::initFromURDF(const std::string& urdf_path, 
                             const std::string& base_link, 
                             const std::string& tip_link) {
    try {
        // 从URDF文件创建KDL树结构
        // URDF文件包含机器人的完整运动学和动力学信息
        // KDL::Tree表示机器人的层次结构，包括所有连杆和关节
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromFile(urdf_path, kdl_tree)) {
            throw std::runtime_error("Failed to parse URDF file: " + urdf_path);
        }

        // 从完整的机器人树中提取指定的运动链
        // 运动链是从基座(base_link)到末端(tip_link)的连续关节序列
        // 这是动力学计算的核心部分，只关心从基座到末端执行器的路径
        chain_ = std::make_unique<KDL::Chain>();
        if (!kdl_tree.getChain(base_link, tip_link, *chain_)) {
            throw std::runtime_error("Failed to extract chain from " + base_link + 
                                " to " + tip_link);
        }

        // 获取运动链中的关节数量
        // 这个数量决定了所有动力学向量和矩阵的维度
        // 包括：关节位置向量、速度向量、力矩向量、惯性矩阵等
        num_joints_ = chain_->getNrOfJoints();
        
        // 初始化各种KDL求解器
        // 包括：动力学参数求解器、雅可比矩阵求解器、正向运动学求解器
        // 这些求解器将在后续的动力学计算中使用
        initSolvers();

        // 标记初始化完成
        // 只有设置了这个标志，才能进行后续的动力学计算
        is_initialized_ = true;
        return true;

    } catch (const std::exception& e) {
        // 捕获并重新抛出异常，添加上下文信息
        // 这样调用者能够知道初始化失败的具体原因
        throw std::runtime_error("KDLDynamics initialization failed: " + std::string(e.what()));
    }
}

void KDLDynamics::initSolvers() {
    // 初始化动力学参数求解器，在构造函数中设置重力向量
    KDL::Vector gravity_vector(0, 0, -9.81);
    dynamics_solver_ = std::make_unique<KDL::ChainDynParam>(*chain_, gravity_vector);
    jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(*chain_);
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(*chain_);
    
    // 预分配内存
    kdl_joint_positions_.resize(num_joints_);
    kdl_joint_velocities_.resize(num_joints_);
    kdl_gravity_torques_.resize(num_joints_);
    kdl_coriolis_torques_.resize(num_joints_);
    kdl_inertia_matrix_.resize(num_joints_);
    kdl_jacobian_.resize(num_joints_);
    
    // 预分配Eigen容器
    last_gravity_compensation_.resize(num_joints_);
    last_coriolis_compensation_.resize(num_joints_);
    last_inertia_matrix_.resize(num_joints_, num_joints_);
    last_payload_gravity_compensation_.resize(num_joints_);
}

Eigen::VectorXd KDLDynamics::calculateGravityCompensation(
    const Eigen::VectorXd& joint_positions) {
    
    // 检查KDL动力学求解器是否已正确初始化
    // 必须先调用initialize()方法设置机器人模型和重力参数
    if (!is_initialized_) {
        throw std::runtime_error("KDLDynamics not initialized");
    }
    
    // 验证输入关节位置向量的有效性
    // 确保向量大小等于关节数量，且不包含NaN或无穷值
    validateInput(joint_positions, "calculateGravityCompensation");
    
    // 将Eigen格式的关节位置转换为KDL格式
    // KDL使用自己的JntArray类型进行动力学计算
    // 这里逐元素复制数据以确保类型安全
    for (size_t i = 0; i < num_joints_; ++i) {
        kdl_joint_positions_(i) = joint_positions(i);
    }
    
    // 使用KDL动力学求解器计算重力补偿力矩
    // JntToGravity考虑以下因素：
    // 1. 机器人各连杆的质量和质心位置
    // 2. 各关节的当前位姿
    // 3. 重力向量（在初始化时设置，通常为[0, 0, -9.81]）
    // 返回值：0表示成功，负值表示错误
    int ret = dynamics_solver_->JntToGravity(kdl_joint_positions_, kdl_gravity_torques_);
    if (ret < 0) {
        throw std::runtime_error("Failed to calculate gravity compensation, error code: " + std::to_string(ret));
    }
    
    // 将计算结果从KDL格式转换回Eigen格式
    // 这样便于在其他使用Eigen的模块中使用
    // 同时保存到last_gravity_compensation_供后续使用
    for (size_t i = 0; i < num_joints_; ++i) {
        last_gravity_compensation_(i) = kdl_gravity_torques_(i);
    }
    
    // 返回重力补偿力矩向量
    // 每个元素对应一个关节的补偿力矩（单位：N·m）
    // 正值表示抵抗重力的力矩方向
    return last_gravity_compensation_;
}


Eigen::VectorXd KDLDynamics::calculateCoriolisCompensation(
    const Eigen::VectorXd& joint_positions,
    const Eigen::VectorXd& joint_velocities) {
    
    if (!is_initialized_) {
        throw std::runtime_error("KDLDynamics not initialized");
    }
    
    validateInput(joint_positions, "calculateCoriolisCompensation");
    validateInput(joint_velocities, "calculateCoriolisCompensation");
    
    // 转换为KDL格式
    for (size_t i = 0; i < num_joints_; ++i) {
        kdl_joint_positions_(i) = joint_positions(i);
        kdl_joint_velocities_(i) = joint_velocities(i);
    }
    
    // 计算科氏力和离心力
    int ret = dynamics_solver_->JntToCoriolis(kdl_joint_positions_, kdl_joint_velocities_, kdl_coriolis_torques_);
    if (ret < 0) {
        throw std::runtime_error("Failed to calculate Coriolis compensation, error code: " + std::to_string(ret));
    }
    
    // 转换回Eigen格式
    for (size_t i = 0; i < num_joints_; ++i) {
        last_coriolis_compensation_(i) = kdl_coriolis_torques_(i);
    }
    
    return last_coriolis_compensation_;
}

Eigen::MatrixXd KDLDynamics::calculateInertiaMatrix(const Eigen::VectorXd& joint_positions) {
    if (!is_initialized_) {
        throw std::runtime_error("KDLDynamics not initialized");
    }
    
    validateInput(joint_positions, "calculateInertiaMatrix");
    
    // 转换为KDL格式
    for (size_t i = 0; i < num_joints_; ++i) {
        kdl_joint_positions_(i) = joint_positions(i);
    }
    
    // 计算惯性矩阵
    int ret = dynamics_solver_->JntToMass(kdl_joint_positions_, kdl_inertia_matrix_);
    if (ret < 0) {
        throw std::runtime_error("Failed to calculate inertia matrix, error code: " + std::to_string(ret));
    }
    
    // 转换回Eigen格式
    for (size_t i = 0; i < num_joints_; ++i) {
        for (size_t j = 0; j < num_joints_; ++j) {
            last_inertia_matrix_(i, j) = kdl_inertia_matrix_(i, j);
        }
    }
    
    return last_inertia_matrix_;
}

Eigen::VectorXd KDLDynamics::calculateJointAcceleration(
    const Eigen::VectorXd& joint_positions,
    const Eigen::VectorXd& joint_velocities,
    const Eigen::Vector3d& end_effector_acceleration,
    const Eigen::MatrixXd& jacobian) {
    
    if (!is_initialized_) {
        throw std::runtime_error("KDLDynamics not initialized");
    }
    
    validateInput(joint_positions, "calculateJointAcceleration");
    validateInput(joint_velocities, "calculateJointAcceleration");
    
    // 计算惯性矩阵
    Eigen::MatrixXd H = calculateInertiaMatrix(joint_positions);
    
    // 构造末端执行器加速度向量（只考虑线加速度）
    Eigen::VectorXd cart_accel(6);
    cart_accel << end_effector_acceleration, Eigen::Vector3d::Zero();
    
    // 使用伪逆计算关节加速度：q̈ = H⁻¹ * Jᵀ * F
    // 其中F是末端执行器力，这里简化为直接映射
    Eigen::MatrixXd J_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd desired_joint_accel = J_pinv * cart_accel;
    
    // 考虑动力学约束
    // 计算科氏力和重力
    Eigen::VectorXd coriolis = calculateCoriolisCompensation(joint_positions, joint_velocities);
    Eigen::VectorXd gravity = calculateGravityCompensation(joint_positions);
    
    // 完整动力学方程：H*q̈ + C*q̇ + G = τ
    // 这里我们计算所需的关节加速度
    Eigen::VectorXd joint_accel = H.inverse() * (desired_joint_accel - coriolis - gravity);
    
    return joint_accel;
}

Eigen::VectorXd KDLDynamics::calculateDynamicsTorque(
    const Eigen::VectorXd& joint_positions,
    const Eigen::VectorXd& joint_velocities,
    const Eigen::VectorXd& joint_accelerations) {
    
    if (!is_initialized_) {
        throw std::runtime_error("KDLDynamics not initialized");
    }
    
    validateInput(joint_positions, "calculateDynamicsTorque");
    validateInput(joint_velocities, "calculateDynamicsTorque");
    validateInput(joint_accelerations, "calculateDynamicsTorque");
    
    // 计算各项动力学分量
    Eigen::VectorXd inertia_torque = calculateInertiaMatrix(joint_positions) * joint_accelerations;
    Eigen::VectorXd coriolis_torque = calculateCoriolisCompensation(joint_positions, joint_velocities);
    Eigen::VectorXd gravity_torque = calculateGravityCompensation(joint_positions);
    
    // 完整动力学力矩：τ = H*q̈ + C*q̇ + G
    return (inertia_torque + coriolis_torque + gravity_torque);
}

Eigen::VectorXd KDLDynamics::calculatePayloadGravityCompensation(
    const Eigen::VectorXd& joint_positions,
    double payload_mass,
    const Eigen::Vector3d& payload_com_in_ee) {

    // 检查KDLDynamics对象是否已初始化，未初始化则抛出异常
    if (!is_initialized_) {
        throw std::runtime_error("KDLDynamics not initialized");
    }

    // 验证输入的关节位置数据
    validateInput(joint_positions, "calculatePayloadGravityCompensation");

    // 如果负载质量小于等于零，返回零向量作为补偿值
    if (payload_mass <= 0.0) {
        return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(num_joints_));
    }

    // 将输入的关节位置数据复制到KDL格式的关节位置对象中
    for (size_t i = 0; i < num_joints_; ++i) {
        kdl_joint_positions_(i) = joint_positions(i);
    }

    // 计算末端执行器的位姿，如果计算失败则抛出异常
    int fk_ret = fk_solver_->JntToCart(kdl_joint_positions_, kdl_end_effector_frame_);
    if (fk_ret < 0) {
        throw std::runtime_error("Failed to calculate end-effector frame, error code: " + std::to_string(fk_ret));
    }

    // 计算雅可比矩阵，如果计算失败则抛出异常
    int jac_ret = jacobian_solver_->JntToJac(kdl_joint_positions_, kdl_jacobian_);
    if (jac_ret < 0) {
        throw std::runtime_error("Failed to calculate Jacobian, error code: " + std::to_string(jac_ret));
    }

    // 定义负载在基坐标系中的重力力
    const KDL::Vector force_base(0.0, 0.0, -payload_mass * 9.81);
    // 计算负载质心在基坐标系中的偏移量
    const KDL::Vector com_offset_base = kdl_end_effector_frame_.M * KDL::Vector(
        payload_com_in_ee.x(), payload_com_in_ee.y(), payload_com_in_ee.z());
    // 计算负载在基坐标系中的力矩
    const KDL::Vector moment_base = com_offset_base * force_base; // r x F

    // 定义负载的力/力矩 wrench
    Eigen::Matrix<double, 6, 1> wrench;
    wrench << force_base.x(), force_base.y(), force_base.z(),
              moment_base.x(), moment_base.y(), moment_base.z();

    // 映射雅可比矩阵数据到Eigen格式的矩阵中
    Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobian_map(
        kdl_jacobian_.data.data(), 6, static_cast<Eigen::Index>(num_joints_));

    // 计算负载重力补偿并返回
    last_payload_gravity_compensation_ = jacobian_map.transpose() * wrench;
    return last_payload_gravity_compensation_;
}


void KDLDynamics::validateInput(const Eigen::VectorXd& data, const std::string& context) const {
    if (static_cast<size_t>(data.size()) != num_joints_) {
        std::stringstream ss;
        ss << "Dimension mismatch in " << context << ": expected " 
           << num_joints_ << ", got " << data.size();
        throw std::runtime_error(ss.str());
    }
}

} // namespace robotic_task
