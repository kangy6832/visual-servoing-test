/**
 * @file robot_kinematics_kdl.cpp
 * @brief 基于KDL和Eigen的机械臂运动学计算实现
 * 
 * @date 2026-01-25
 */
#include "robot_kinematics_kdl.hpp"
#include <sstream>
#include <iomanip>
#include <Eigen/src/IterativeLinearSolvers/IterativeSolverBase.h>

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

        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(*chain_); // 正运动学位置求解器
        jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(*chain_); // 雅克比矩阵求解器
        jacobian_dot_solver_ = std::make_unique<KDL::ChainJntToJacDotSolver>(*chain_); // 雅克比矩阵导数求解器
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
        int result = fk_solver_->JntToCart(kdl_joint_positions_, kdl_frame); // JntToCart 关节角度->末端位置
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
        return jacobian;
    }
    
    Eigen::MatrixXd RobotArmKinematics::computeJacobianDerivative(
        const Eigen::VectorXd& joint_positions, 
        const Eigen::VectorXd& joint_velocities
    ){
        validateInput(joint_positions, "computeJacobianDerivative:positions");
        validateInput(joint_velocities, "computeJacobianDerivate:velocities");
        checkFinite(joint_positions, "joint_positions");
        checkFinite(joint_velocities, "joint_velocities");

        // 转换为KDL格式
        for (size_t i = 0 ; i < num_joints_ ; ++i){
            kdl_joint_positions_(i) = joint_positions[i];
            kdl_joint_velocities_(i) = joint_velocities[i];
        }

        //计算雅克比矩阵导数
        KDL::Jacobian jacobian_dot(num_joints_);

        int result = jacobian_dot_solver_->JntToJacDot(KDL::JntArrayVel(kdl_joint_positions_, kdl_joint_velocities_), jacobian_dot); // JntToJacDot 计算雅可比矩阵导数与关节速度的乘积
        if(result < 0){
            throw KinematicSolverException(
                result, "ChainJntToJacDotSolver"
            );
        }

        // 转换为Eigen格式
        Eigen::MatrixXd jacobian_dot_eigen(6, num_joints_);
        for (unsigned int i = 0 ; i < 6 ; ++i){
            for(unsigned int j = 0 ; j < num_joints_ ; ++j){
                jacobian_dot_eigen(i, j) = jacobian_dot(i, j);
            }
        }

        last_jacobian_dot_ = jacobian_dot_eigen;
        return jacobian_dot_eigen;
    }


    // =========================================================================
    // 速度计算
    // =========================================================================

    CartesianTwist RobotArmKinematics::forwardVelocityKinematics(
        const Eigen::VectorXd& joint_positions, 
        const Eigen::VectorXd& joint_velocities
    ){
        validateInput(joint_positions, "forwardVelocityKinematics:positions");
        validateInput(joint_velocities, "forwardVelocitKinematics:velocities");
        checkFinite(joint_velocities, "joint_velocities");

        // 计算雅克比矩阵
        Eigen::MatrixXd jacobian = computeJacobian(joint_positions);

        // 检查奇异位形
        if (config_.enable_singularity_check){
            SingularityAnalysis analysis = analyzeSingularity(joint_positions);
            if(analysis.is_near_singular){
                throw SingularityException(analysis.smallest_singular_value, config_.singularity_threshold);
            }
        }

        // 计算末端速度：v = J * q_dot
        Eigen::VectorXd cartesian_velocity = jacobian * joint_velocities;
        return CartesianTwist::fromVector(cartesian_velocity); // fromVector Eigen向量转KDL
    }

    Eigen::VectorXd RobotArmKinematics::inverseVelocityKinematics(
        const Eigen::VectorXd& joint_positions, 
        const CartesianTwist& desired_velocity
    ){
        validateInput(joint_positions, "inverseVelocityKinematics");
        checkFinite(joint_positions, "joint_positions");

        Eigen::MatrixXd jacobian = computeJacobian(joint_positions);
        Eigen::VectorXd v_desired = desired_velocity.toVector();

        // 使用奇异值分解计算伪逆
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // 检查是否需要阻尼
        double damping = config_.use_damped_least_squares ? config_.damping_factor : 0.0;
        return svd.solve(v_desired);
    }

    Eigen::VectorXd RobotArmKinematics::dampedLeastSquaresInverseVelocity(
        const Eigen::VectorXd& joint_positions, 
        const CartesianTwist& desired_velocity, 
        double damping_factor
    ){
        if(damping_factor < 0){
            damping_factor = config_.damping_factor;
        }

        validateInput(joint_positions, "dampedLeastSquaresInverseVelocity");
        checkFinite(joint_positions, "joint_positions");

        Eigen::MatrixXd jacobian = computeJacobian(joint_positions);
        Eigen::VectorXd v_desired = desired_velocity.toVector();

        // 阻尼最小二乘法
        Eigen::MatrixXd jjt = jacobian * jacobian.transpose();

        // 添加阻尼项
        for(int i = 0 ; i < 6 ; ++i){
            jjt(i, i) += damping_factor * damping_factor;
        }

        Eigen::VectorXd temp = jjt.inverse() * v_desired;
        return jacobian.transpose() * temp;
    }

    // ============================================================================
    // 加速度计算
    // ============================================================================

    CartesianAcceleration RobotArmKinematics::forwardAccelerationKinematics(
        const Eigen::VectorXd& joint_positions, 
        const Eigen::VectorXd& joint_velocities, 
        const Eigen::VectorXd& joint_accelerations
    ){
        validateInput(joint_positions, "forwardAccelerationKinematics:positions");
        validateInput(joint_velocities, "forwardAccelerationKinematics:velocities");
        validateInput(joint_accelerations, "forwardAccelerationKinematics:accelerations");
        checkFinite(joint_positions, "joint_positions");
        checkFinite(joint_velocities, "joint_velocities");
        checkFinite(joint_accelerations, "joint_accelerations");

        // 计算雅克比矩阵和导数
        Eigen::MatrixXd jacobian = computeJacobian(joint_positions);
        Eigen::MatrixXd jacobian_dot = computeJacobianDerivative(
            joint_positions, joint_velocities);

        // 计算末端加速度
        Eigen::VectorXd accel = jacobian * joint_accelerations + jacobian_dot * joint_velocities;
        return CartesianAcceleration::fromVector(accel);
    }


    // ==========================================================================================
    // 完整运动状态
    // ==========================================================================================

    CartesianMotionState RobotArmKinematics::computeFullCartesianState(
        const Eigen::VectorXd& joint_positions, 
        const Eigen::VectorXd& joint_velocities, 
        const Eigen::VectorXd& joint_accelerations
    ){
        CartesianMotionState state;

        try {
            // 计算位姿
            state.pose = computeForwardKinematics(joint_positions);

            // 计算雅克比矩阵
            state.jacobian = computeJacobian(joint_positions);

            // 分析奇异位形
            SingularityAnalysis singularity = analyzeSingularity(joint_positions);
            state.near_singularity = singularity.is_near_singular;
            state.condition_number = singularity.condition_number;

            // 计算速度（捕获异常以继续计算加速度）
            try{
                state.velocity = forwardVelocityKinematics(joint_positions, joint_velocities);
            } catch (const SingularityException&){
                state.velocity = CartesianTwist();
                state.status_message = "Near singularity - velocity computation skipped";
            }

            // 计算加速度
            try{
                state.acceleration = forwardAccelerationKinematics(
                    joint_positions, joint_velocities, joint_accelerations);
            } catch (const std::exception&){
                state.acceleration = CartesianAcceleration();
                if(!state.status_message.empty()){
                    state.status_message += ";";
                }
                state.status_message += "Acceleration computation failed";
            }

            if(!state.near_singularity){
                state.status_message = "OK";
            }
        } catch (const std::exception& e){
            std::lock_guard<std::mutex>lock(status_mutex_);
            status_message_ = std::string("Error: ") + e.what();
            error_count_++;
        }

        return state;
    }


    // =====================================================================================
    // 奇异值分析
    // =====================================================================================

    SingularityAnalysis RobotArmKinematics::analyzeSingularity(const Eigen::VectorXd& joint_positions){
        SingularityAnalysis analysis;
        Eigen::MatrixXd jacobian = computeJacobian(joint_positions);

        // 奇异值分析
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU); // JacobiSVD 计算奇异值分解
        analysis.singular_values.resize(svd.singularValues().size());

        for (size_t i = 0 ; i < svd.singularValues().size() ; ++i){
            analysis.singular_values[i] = svd.singularValues()(i);
        }

        // 计算条件数
        if (analysis.singular_values.empty()){
            analysis.condition_number = 0.0;
        } else {
            double max_sv = analysis.singular_values[0];
            double min_sv = analysis.singular_values.back();
            analysis.smallest_singular_value = min_sv;
            analysis.condition_number = max_sv / (min_sv + 1e-10);
        }

        // 检查是否接近奇异位形
        analysis.is_near_singular = analysis.smallest_singular_value < config_.singularity_threshold;

        // 计算可操作性
        Eigen::MatrixXd jjt = jacobian * jacobian.transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(jjt.topLeftCorner(3, 3));
        analysis.directional_manipulability = solver.eigenvalues();
        return analysis;
    }

    bool RobotArmKinematics::isNearSingularity(const Eigen::VectorXd& joint_positions){
        SingularityAnalysis analysis = analyzeSingularity(joint_positions);
        return analysis.is_near_singular;
    }

    double RobotArmKinematics::computeManipulabilityIndex(const Eigen::VectorXd& joint_positions){
        Eigen::MatrixXd jacobian = computeJacobian(joint_positions);
        Eigen::MatrixXd jjt = jacobian * jacobian.transpose();
        return std::sqrt(jjt.determinant());
    }
} // namespace RobotKinematicsKDL



