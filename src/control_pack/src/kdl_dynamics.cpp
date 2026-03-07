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
        // 从URDF文件创建KDL树
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromFile(urdf_path, kdl_tree)) {
            throw std::runtime_error("Failed to parse URDF file: " + urdf_path);
        }

        // 从树中提取链
        chain_ = std::make_unique<KDL::Chain>();
        if (!kdl_tree.getChain(base_link, tip_link, *chain_)) {
            throw std::runtime_error("Failed to extract chain from " + base_link + 
                                " to " + tip_link);
        }

        num_joints_ = chain_->getNrOfJoints();
        
        // 初始化求解器
        initSolvers();

        is_initialized_ = true;
        return true;

    } catch (const std::exception& e) {
        throw std::runtime_error("KDLDynamics initialization failed: " + std::string(e.what()));
    }
}

void KDLDynamics::initSolvers() {
    // 初始化动力学参数求解器，在构造函数中设置重力向量
    KDL::Vector gravity_vector(0, 0, -9.81);
    dynamics_solver_ = std::make_unique<KDL::ChainDynParam>(*chain_, gravity_vector);
    
    // 预分配内存
    kdl_joint_positions_.resize(num_joints_);
    kdl_joint_velocities_.resize(num_joints_);
    kdl_gravity_torques_.resize(num_joints_);
    kdl_coriolis_torques_.resize(num_joints_);
    kdl_inertia_matrix_.resize(num_joints_);
    
    // 预分配Eigen容器
    last_gravity_compensation_.resize(num_joints_);
    last_coriolis_compensation_.resize(num_joints_);
    last_inertia_matrix_.resize(num_joints_, num_joints_);
}

Eigen::VectorXd KDLDynamics::calculateGravityCompensation(
    const Eigen::VectorXd& joint_positions) {
    
    if (!is_initialized_) {
        throw std::runtime_error("KDLDynamics not initialized");
    }
    
    validateInput(joint_positions, "calculateGravityCompensation");
    
    // 转换为KDL格式
    for (size_t i = 0; i < num_joints_; ++i) {
        kdl_joint_positions_(i) = joint_positions(i);
    }
    
    // 计算重力力矩（重力向量在初始化时已设置）
    int ret = dynamics_solver_->JntToGravity(kdl_joint_positions_, kdl_gravity_torques_);
    if (ret < 0) {
        throw std::runtime_error("Failed to calculate gravity compensation, error code: " + std::to_string(ret));
    }
    
    // 转换回Eigen格式
    for (size_t i = 0; i < num_joints_; ++i) {
        last_gravity_compensation_(i) = kdl_gravity_torques_(i);
    }
    
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
    return inertia_torque + coriolis_torque + gravity_torque;
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
