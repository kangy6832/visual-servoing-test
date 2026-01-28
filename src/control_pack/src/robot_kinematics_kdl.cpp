/**
 * @file robot_kinematics_kdl.cpp
 * @brief 基于KDL和Eigen的机械臂运动学计算实现
 * 
 * @date 2026-01-25
 */
#include "robot_kinematics_kdl.hpp"
#include <sstream>
#include <iomanip>

namespace RobotKinematicsKDL{

    // ============================================================
    // 构造函数与析构函数
    // ============================================================

    RobotArmKinematics::RobotArmKinematics(size_t num_joints) : 
        num_joints_(num_joints){
            preallocateMemory();
        }

    RobotArmKinematics::~RobotArmKinematics() = default;


    // ============================================================
    // 内存预分配
    // ============================================================

    void RobotArmKinematics::preallocateMemory(){

        // 预分配 KDL 数据容器
        kdl_joint_positions_ = KDL::JntArray(num_joints_);
        kdl_joint_velocities_ = KDL::JntArray(num_joints_);
        kdl_joint_accelerations_ = KDL::JntArray(num_joints_);
        kdl_jacobian_ = KDL::Jacobian(num_joints_);

        // 预分配Eigen矩阵
        last_jacobian_ = Eigen::MatrixXd(6, num_joints_);  // 行数（6）， 列数（num_joints_)
        last_jacobian_dot_ = Eigen::MatrixXd(6, num_joints_);

        // 初始化默认关节限位
        joint_limits_.resize(num_joints_);
    }
    

    // ============================================================
    // 求解器初始化
    // ============================================================

    void RobotArmKinematics::initSolvers() {
        if(!chain_ || chain_->getNrOfJoints() == 0){  // getNrOfJoints 获取机械臂关节数量
            throw std::runtime_error("Cannot initialize solvers: Invalid chain");
        }

        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(*chain_);
        jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(*chain_);
        jocobian_dot_solver_ = std::make_unique<KDL::ChainJntToJacDotSolver>(*chain_);
        is_initialized_ = true;
    }


    // ============================================================
    // 初始化方法
    // ============================================================

    bool RobotArmKinematics::initFromDHParams(const std::vector<DHParameters>& dh_params){
        try{
            chain_ = std::make_unique<KDL::Chain>();
        
            for(const auto& dh : dh_params){
                chain_->addSegment(dh.toKDLSegment());
            }

            num_joints_ = chain_->getNrOfJoints();
            preallocateMemory();
            initSolvers();
            std::lock_guard<std::mutex> lock(status_mutex_);
            std::ostringstream ss;
            ss << "Initialized from " << dh_params.size() << " DH parameters," 
                << num_joints_ << " joints";
            status_message_ = ss.str();
            return true;

        } catch (const std::exception& e){
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_message_ = std::string("Init failed: ") + e.what();
            error_count_ ++ ;
            return false ;
        }
    }

    bool RobotArmKinematics::initStandard6DOF() {
        std::vector<DHParameters> dh_params = {
            DHParameters(0.0, -PI/2, 0.0, 0.0, true),   // 关节1
            DHParameters(0.0, PI/2, 0.0, 0.0, true),   // 关节2
            DHParameters(0.612, 0.0, 0.0, 0.0, true),  // 关节3
            DHParameters(0.0, -PI/2, 0.5723, 0.0, true),   // 关节4
            DHParameters(0.0, PI/2, 0.0, 0.0, true),   // 关节5
            DHParameters(0.0, 0.0, 0.0, 0.0, true)    // 关节6
        };

        return initFromDHParams(dh_params);
    }

    bool RobotArmKinematics::loadFromURDF(const std::string& urdf_path, 
                                          const std::string& base_link, 
                                          const std::string& tip_link){
    // 使用依赖
    }

    void RobotArmKinematics::setChain(const KDL::Chain& chain){
        chain_ = std::make_unique<KDL::Chain>(chain);
        num_joints_ = chain_->getNrOfJoints();
        preallocateMemory();
        initSolvers();
    }

    void RobotArmKinematics::setJointLimits(const JointLimits& limits){
        if(limits.min_positions.size() != num_joints_){
            throw DimensionMismatchException(
                static_cast<int>(num_joints_), static_cast<int>(limits.min_positions.size()),"JointLimits::resize"
            );
        }
        joint_limits_ = limits;
    }

    void RobotArmKinematics::setConfig(const ControllerConfig& config){
        config_ = config;
    }

    // ============================================================
    // 关节限位验证
    // ============================================================

    void RobotArmKinematics::validateJointLimits(const Eigen::VectorXd& positions) const {
        for(size_t i = 0; i < positions.size(); ++i){
            if(positions[i] < joint_limits_.min_positions[i] || 
               positions[i] > joint_limits_.max_positions[i]){
                throw JointLimitException(
                    static_cast<int>(i), positions[i], joint_limits_.min_positions[i], joint_limits_.max_positions[i]
                );
            }
        }
    }


    // ============================================================
    // 正向运动学
    // ============================================================

    EndEffectorPose RobotArmKinematics::computeForwardKinematics(
        const Eigen::VectorXd& joint_positions
    ){
        validateInput(joint_positions, "computeForwardKinematics");
        checkFinite(joint_positions, "joint_positions");
        validateJointLimits(joint_positions);

        // 转换为KDL格式
        for(size_t i = 0 ; i < num_joints_ ; ++i){
            kdl_joint_positions_(i) = joint_positions[i];
        }

        // 计算正向运动学
        KDL::Frame kdl_frame;
        int result = fk_solver_->JntToCart(kdl_joint_positions_, kdl_frame);
        if(result < 0){
            throw KinematicSolverException(
                result, "ChainFkSolverPos_recursive"
            );
        }

        // 转换为Eigen格式
        EndEffectorPose pose ;
        pose.position = Eigen::Vector3d(
            kdl_frame.p.data[0],
            kdl_frame.p.data[1],
            kdl_frame.p.data[2]
        );

        // 转换旋转矩阵为四元数
        Eigen::Matrix3d rotation_matrix;
        for(int i = 0 ; i < 3 ; ++i){
            for(int j = 0 ; j < 3 ; ++j){
                rotation_matrix(i, j) = kdl_frame.M.data[i * 3 + j];
            }
        }
        pose.orientation = Eigen::Quaterniond(rotation_matrix);

        return pose;
    }

    Eigen::Matrix4d RobotArmKinematics::computeForwardKinematicsMatrix(
        const Eigen::VectorXd& joint_positions
    ){
        EndEffectorPose pose = computeForwardKinematics(joint_positions);
        return pose.toHomogeneousMatrix();
    }


    // ============================================================
    // 雅可比矩阵计算
    // ============================================================

    Eigen::MatrixXd RobotArmKinematics::computeJacobian(
        const Eigen::VectorXd& joint_positions
    ){
        validateInput(joint_positions, "computeJacobian");
        checkFinite(joint_positions, "joint_positions");

        // 转换为KDL格式
        for(size_t i = 0 ; i < num_joints_ ; ++i){
            kdl_joint_positions_(i) = joint_positions[i];
        }

        // 计算雅克比矩阵
        int result = jacobian_solver_->JntToJac(kdl_joint_positions_, kdl_jacobian_);
        if(result < 0){
            throw KinematicSolverException(
                result, "ChainJntToJacSolver"
            );
        }

        // 转换为Eigen格式
        Eigen::MatrixXd jacobian(6, num_joints_);
        for (unsigned int i = 0 ; i < 6 ; ++i){
            for(unsigned int j = 0 ; j < num_joints_ ; ++j){
                jacobian(i, j) = kdl_jacobian_(i, j);
            }
        }

        last_jacobian_ = jacobian;
    }
    


}



