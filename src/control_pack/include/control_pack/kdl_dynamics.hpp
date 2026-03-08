/**
 * @file kdl_dynamics.hpp
 * @brief KDL动力学计算类
 * 
 * 使用KDL的ChainDynParam进行机器人动力学计算，包括重力、科氏力和惯性矩阵
 * 
 * @version 1.0.0
 * @date 2026-03-07
 */

#ifndef KDL_DYNAMICS_HPP
#define KDL_DYNAMICS_HPP

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <Eigen/Dense>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <string>

namespace robotic_task {

/**
 * @brief KDL动力学计算类
 * 
 * 该类封装了使用KDL进行机器人动力学计算的功能，包括：
 * - 重力补偿计算 G(q)
 * - 科氏力和离心力计算 C(q,q̇)
 * - 惯性矩阵计算 H(q)
 * - 关节力矩计算
 */
class KDLDynamics {
public:
    /**
     * @brief 构造函数
     */
    KDLDynamics();

    /**
     * @brief 析构函数
     */
    ~KDLDynamics();

    /**
     * @brief 从URDF文件初始化动力学参数
     * 
     * @param urdf_path URDF文件路径
     * @param base_link 基座链接名称
     * @param tip_link 末端链接名称
     * @return true 初始化成功
     */
    bool initFromURDF(const std::string& urdf_path, 
                     const std::string& base_link = "joint1", 
                     const std::string& tip_link = "joint6");

    /**
     * @brief 计算重力补偿力矩
     * 
     * @param joint_positions 关节位置
     * @return 重力补偿力矩
     */
    Eigen::VectorXd calculateGravityCompensation(
        const Eigen::VectorXd& joint_positions
    );


    /**
     * @brief 计算科氏力和离心力补偿
     * 
     * @param joint_positions 关节位置
     * @param joint_velocities 关节速度
     * @return 科氏力和离心力补偿力矩
     */
    Eigen::VectorXd calculateCoriolisCompensation(
        const Eigen::VectorXd& joint_positions,
        const Eigen::VectorXd& joint_velocities
    );

    /**
     * @brief 计算惯性矩阵
     * 
     * @param joint_positions 关节位置
     * @return 惯性矩阵 H(q)
     */
    Eigen::MatrixXd calculateInertiaMatrix(const Eigen::VectorXd& joint_positions);

    /**
     * @brief 计算关节加速度（基于末端恒定加速度）
     * 
     * @param joint_positions 关节位置
     * @param joint_velocities 关节速度
     * @param end_effector_acceleration 末端执行器恒定加速度
     * @param jacobian 雅可比矩阵
     * @return 关节加速度
     */
    Eigen::VectorXd calculateJointAcceleration(
        const Eigen::VectorXd& joint_positions,
        const Eigen::VectorXd& joint_velocities,
        const Eigen::Vector3d& end_effector_acceleration,
        const Eigen::MatrixXd& jacobian
    );

    /**
     * @brief 计算完整的动力学力矩
     * 
     * @param joint_positions 关节位置
     * @param joint_velocities 关节速度
     * @param joint_accelerations 关节加速度
     * @return 完整动力学力矩
     */
    Eigen::VectorXd calculateDynamicsTorque(
        const Eigen::VectorXd& joint_positions,
        const Eigen::VectorXd& joint_velocities,
        const Eigen::VectorXd& joint_accelerations
    );

    /**
     * @brief 计算末端附加载荷的重力补偿力矩
     *
     * @param joint_positions 关节位置
     * @param payload_mass 负载质量(kg)
     * @param payload_com_in_ee 负载质心在末端坐标系中的偏移(m)
     * @return 关节补偿力矩
     */
    Eigen::VectorXd calculatePayloadGravityCompensation(
        const Eigen::VectorXd& joint_positions,
        double payload_mass,
        const Eigen::Vector3d& payload_com_in_ee = Eigen::Vector3d::Zero()
    );

    /**
     * @brief 检查是否已初始化
     * @return true 已初始化
     */
    bool isInitialized() const { return is_initialized_; }

    /**
     * @brief 获取关节数量
     * @return 关节数量
     */
    size_t getNumJoints() const { return num_joints_; }

private:
    /**
     * @brief 初始化KDL动力学求解器
     */
    void initSolvers();

    /**
     * @brief 验证输入维度
     */
    void validateInput(const Eigen::VectorXd& data, const std::string& context) const;

    // KDL组件
    std::unique_ptr<KDL::Chain> chain_;
    std::unique_ptr<KDL::ChainDynParam> dynamics_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

    // 内部状态
    size_t num_joints_;
    bool is_initialized_;

    // KDL数据容器（预分配以避免运行时分配）
    mutable KDL::JntArray kdl_joint_positions_;
    mutable KDL::JntArray kdl_joint_velocities_;
    mutable KDL::JntArray kdl_gravity_torques_;
    mutable KDL::JntArray kdl_coriolis_torques_;
    mutable KDL::JntSpaceInertiaMatrix kdl_inertia_matrix_;
    mutable KDL::Jacobian kdl_jacobian_;
    mutable KDL::Frame kdl_end_effector_frame_;

    // Eigen数据容器
    mutable Eigen::VectorXd last_gravity_compensation_;
    mutable Eigen::VectorXd last_coriolis_compensation_;
    mutable Eigen::MatrixXd last_inertia_matrix_;
    mutable Eigen::VectorXd last_payload_gravity_compensation_;
};

} // namespace robotic_task

#endif // KDL_DYNAMICS_HPP
