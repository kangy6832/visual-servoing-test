/**
 * @file robot_kinematics_kdl.hpp
 * @brief 基于KDL和Eigen的机械臂运动学计算库
 * 
 * 该库提供了使用KDL进行正向运动学计算、使用Eigen进行矩阵运算的
 * 完整机械臂运动学解决方案，包括速度、加速度计算和奇异性检测。
 * 
 * @version 1.0.0
 * @date 2026-01-22
 */

#ifndef ROBOT_KINEMATICS_KDL_HPP
#define ROBOT_KINEMATICS_KDL_HPP

#include <eigen3/Eigen/Core>

#include <cstddef>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <kdl/chain.hpp>  // 机器人链结构
#include <Eigen/Core>
#include <kdl/chainfksolverpos_recursive.hpp>  // 正运动学求解器
#include <kdl/chainjnttojacsolver.hpp>  //  雅可比矩阵求解器
#include <kdl/chainjnttojacdotsolver.hpp>  // 雅可比矩阵导数求解器
#include <kdl/frames.hpp>  // 位姿变换框架
#include <kdl/jacobian.hpp>  // 雅可比矩阵
#include <Eigen/Dense>  // 密集矩阵运算
#include <Eigen/SVD>  // 奇异值分解
#include <kdl/jntarray.hpp>
#include <kdl/segment.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>  // 标准异常类
#include <memory>
#include <optional>  // 可选值
#include <functional>
#include <mutex>  // 互斥锁
#include <atomic>  // 原子操作
#include <cmath>
#include <limits>  // 数值极限

/**
 * KDL 逆向速度运动学求解器（Jacobian 伪逆法）
 * 用于：由末端 Twist 计算关节速度 dq
 * 原理：dq = J⁺ * v
 * 注意：在奇异位形附近数值不稳定
 */
#include <kdl/chainiksolvervel_pinv.hpp>   // IK 速度求解器

/**
 * KDL 位姿级逆运动学求解器（Newton–Raphson 数值迭代）
 * 
 * ⚠ 重要说明：ChainIkSolverPos_NR 本身【不能单独工作】它内部必须依赖：
 *   - 一个正向运动学求解器（FK）
 *   - 一个速度级 IK 求解器（ChainIkSolverVel_*）
 * 
 * 数学形式： q(k+1) = q(k) + J⁺(q) * (x_desired - FK(q))
 * 
 * 因此它只是一个“框架调度器”，不是完整 IK 算法
 */
#include <kdl/chainiksolverpos_nr.hpp>     // IK 位姿求解器（Newton-Raphson）

#include <fstream>
#include <streambuf>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>


namespace RobotKinematicsKDL {
    // ============================================================
    // 基础常量定义
    // ============================================================
    constexpr double PI = 3.14159265358979323846;
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;
    constexpr double DEFAULT_SINGULARITY_THRESHOLD = 1e-4;
    constexpr double DEFAULT_MAX_VELOCITY = 10.0;       // m/s
    constexpr double DEFAULT_MAX_ANGULAR_VELOCITY = 10.0; // rad/s
    constexpr double DEFAULT_MAX_ACCELERATION = 100.0;  // m/s²
    constexpr double DEFAULT_MAX_ANGULAR_ACCELERATION = 100.0; // rad/s²
    constexpr double POSITION_TOLERANCE = 1e-6;
    constexpr double ORIENTATION_TOLERANCE = 1e-6;

    // ============================================================
    // 自定义异常类
    // ============================================================

    /**
     * @brief 维度不匹配异常
     */
     class DimensionMismatchException : public std::runtime_error {
        public:
            explicit DimensionMismatchException(const std::string& msg) : std::runtime_error(msg){}
            DimensionMismatchException(int expected, int actual, const std::string& context) : 
                std::runtime_error(std::string("Dimension mismatch in ") + context + std::string(": expected ") + std::to_string(expected) + std::string(", got ") + std::to_string(actual)){}
     };


     /**
     * @brief 运动学求解器异常
     */
     class KinematicSolverException : public std::runtime_error{
        public:
            explicit KinematicSolverException(const std::string& msg) : std::runtime_error(msg){}
            KinematicSolverException(int error_code, const std::string& solver_name) : 
                std::runtime_error("KDL " + solver_name + " error code: " + 
                std::to_string(error_code)){}
     };


     /**
     * @brief 关节限位异常
     */
     class JointLimitException : public std::runtime_error{
        public:
            explicit JointLimitException(int joint_index, double value, double min_limit, double max_limit) : 
            std::runtime_error(std::string("Joint ") + std::to_string(joint_index) + std::string(" value ") + 
            std::to_string(value) + " exceeds limits [" + std::to_string(min_limit) + ", " + 
            std::to_string(max_limit) + "]"){}
     };


     /**
     * @brief 奇异位形异常
     */
     class SingularityException : public std::runtime_error {
        public:
            explicit SingularityException(const std::string& msg) : std::runtime_error(msg){}
            SingularityException(double smallest_singular_value, double threshold) : 
                std::runtime_error(std::string("Near-singular configuration detected: ") + 
                std::string("smallest singular value (") + std::to_string(smallest_singular_value) + 
                ") below threshold (" + std::to_string(threshold) + std::string(")")){}
     };


     /**
     * @brief 未初始化异常
     */
     class NotInitializedException : public std::runtime_error{
        public :
            explicit NotInitializedException(const std::string& component) : std::runtime_error(component + 
                " not initialized. Call init() first."){}
     };


    // ============================================================
    // 配置结构体
    // ============================================================
    
    /**
     * @brief 关节限位配置
     */
     struct JointLimits {
        std::vector<double> min_positions;
        std::vector<double> max_positions;
        std::vector<double> min_velocities;
        std::vector<double> max_velocities;
        std::vector<double> min_accelerations;
        std::vector<double> max_accelerations;

        JointLimits() = default;

        explicit JointLimits(size_t num_joints){
            resize(num_joints);
        }

        void resize (size_t num_joints){
            min_positions.resize(num_joints, -PI);
            max_positions.resize(num_joints, PI);
            min_velocities.resize(num_joints, -DEFAULT_MAX_VELOCITY);
            max_velocities.resize(num_joints, DEFAULT_MAX_VELOCITY);
            min_accelerations.resize(num_joints, -DEFAULT_MAX_ACCELERATION);
            max_accelerations.resize(num_joints, DEFAULT_MAX_ACCELERATION);
        }
        
        bool validatePositions(const Eigen::VectorXd& positions) const {
            if (positions.size() != min_positions.size()) return false;
            for(size_t i = 0 ; i < positions.size() ; ++i){
                if (positions[i] < min_positions[i] || positions[i] > max_positions[i]) return false;
            }
            return true;
        }

        bool validateVelocities(const Eigen::VectorXd& velocities) const {
            if (velocities.size() != min_velocities.size()) return false;
            for(size_t i = 0 ; i < velocities.size() ; ++i){
                if (velocities[i] < min_velocities[i] || velocities[i] > max_velocities[i]) return false;
            }
            return true;
        }

        bool validateAccelerations(const Eigen::VectorXd& accelerations) const {
            if (accelerations.size() != min_accelerations.size()) return false;
            for(size_t i = 0 ; i < accelerations.size() ; ++i){
                if (accelerations[i] < min_accelerations[i] || accelerations[i] > max_accelerations[i]) return false;
            }
            return true;
        }
     };

     /**
     * @brief 控制器配置
     */
     struct ControllerConfig{
        double singularity_threshold = DEFAULT_SINGULARITY_THRESHOLD;
        bool enable_singularity_check = true; // 奇异点检测
        bool enable_joint_limit_check = true;
        bool enable_nan_check = true;
        bool use_damped_least_squares = false;
        double damping_factor = 0.01;
        int max_iterations = 100;
        double convergence_threshold = 1e-4;
        bool verbose = false;
        ControllerConfig() = default;
     };


    // ============================================================
    // 笛卡尔空间运动状态结构体
    // ============================================================

    /**
     * @brief 六维空间速度向量（Twist）
     */
    struct CartesianTwist {
        Eigen::Vector3d linear;  // 线速度 （vx, vy, vz) [m/s]
        Eigen::Vector3d angular ;  // 角速度 (wx, wy, wz) [rad/s]

        CartesianTwist() : linear(Eigen::Vector3d::Zero()), angular(Eigen::Vector3d::Zero()) {}

        CartesianTwist(double vx, double vy, double vz, double wx, double wy, double wz) : 
            linear(vx, vy, vz), angular(wx, wy, wz){}

        CartesianTwist(const Eigen::Vector3d& lin, const Eigen::Vector3d& ang) : 
            linear(lin), angular(ang){}

        Eigen::VectorXd toVector() const{
            Eigen::VectorXd result(6);
            result << linear, angular;
            return result;
        }

        static CartesianTwist fromVector(const Eigen::VectorXd& v){
            if(v.size() != 6){
                throw DimensionMismatchException(6, v.size(), 
                    "CartesianTwist::fromVector");
            }
            return CartesianTwist(Eigen::Vector3d(v[0], v[1], v[2]), Eigen::Vector3d(v[3], v[4], v[5]));
        }

        double linearNorm() const {return linear.norm();}
        double angularNorm() const {return angular.norm();}
        double totalNorm() const {return toVector().norm();}
        bool isValid() const {return std::isfinite(linear.norm()) && std::isfinite(angular.norm());}
    };

    /**
    * @brief 六维空间加速度向量
    */
    struct CartesianAcceleration{
        Eigen::Vector3d linear;  // 线加速度 (ax, ay, az) 
        Eigen::Vector3d angular; // 角加速度 (alphax, alphay, alphaz)

        CartesianAcceleration() : linear(Eigen::Vector3d::Zero()), angular(Eigen::Vector3d::Zero()){}

        CartesianAcceleration(double ax, double ay, double az, double alphax, double alphay, double alphaz) : 
            linear(ax, ay, az), angular(alphax, alphay, alphaz){}

        CartesianAcceleration(const Eigen::Vector3d& lin, const Eigen::Vector3d& ang) : linear(lin), angular(ang){}
        
        Eigen::VectorXd toVector() const{
            Eigen::VectorXd result(6);  // 六维向量
            result << linear, angular;  // 拼接
            return result;
        }
        
        static CartesianAcceleration fromVector(const Eigen::VectorXd& a){
            if(a.size() != 6){
                throw DimensionMismatchException(6, a.size(), "CartesianAcceleration::fromVector");
            }
            return CartesianAcceleration(Eigen::Vector3d(a[0], a[1], a[2]), 
                Eigen::Vector3d(a[3], a[4], a[5]));
        }

        double linearNorm() const {return linear.norm();}

        double angularNorm() const {return angular.norm();}

        double totalNorm() const {return toVector().norm();}

        bool isValid() const {
            return std::isfinite(linear.norm()) && std::isfinite(angular.norm());
        }
    };

    /**
    * @brief 末端执行器位姿
    */
    struct EndEffectorPose {
        Eigen::Vector3d position;  // 位置
        Eigen::Quaterniond orientation; // 姿态

        EndEffectorPose() : position(Eigen::Vector3d::Zero()),
                            orientation(Eigen::Quaterniond::Identity()){}

        EndEffectorPose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient) : 
            position(pos), orientation(orient) {}
        
        EndEffectorPose(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot_matrix) : 
            position(pos), orientation(Eigen::Quaterniond(rot_matrix)){}

        Eigen::Matrix4d toHomogeneousMatrix() const {  // 4×4 齐次变换矩阵
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = orientation.toRotationMatrix();  // 四元是转旋转矩阵
            T.block<3, 1>(0, 3) = position;
            return T;
        }

        static EndEffectorPose fromHomogeneousMatrix(const Eigen::Matrix4d& T) {  
            EndEffectorPose pose;
            pose.position = T.block<3, 1>(0, 3);
            pose.orientation = Eigen::Quaterniond(T.block<3, 3>(0, 0));
            return pose;
        }

        Eigen::Vector3d toRPY() const {  // 四元数转欧拉角
            Eigen::Matrix3d R = orientation.toRotationMatrix(); // 四元数转旋转矩阵
            double roll = std::atan2(R(2, 1), R(2, 2));
            double pitch = std::atan2(-R(2,0), std::sqrt(R(2, 1)*R(2, 1) + R(2, 2)*R(2, 2)));
            double yaw = std::atan2(R(1, 0), R(0, 0));
            return Eigen::Vector3d(roll, pitch, yaw);
            // return orientation.toRotationMatrix().eulerAngles(2, 1, 0);  // 在奇异点附近数值不稳定
        }
    
        bool isValid() const {
            return (std::isfinite(position.norm()) && std::isfinite(orientation.norm()));
        }
    };

    /**
     * @brief 完整的笛卡尔空间运动状态
     */
    struct CartesianMotionState{
        EndEffectorPose pose;
        CartesianTwist velocity;
        CartesianAcceleration acceleration;
        bool near_singularity = false;
        double condition_number = 0.0;
        Eigen::MatrixXd jacobian;
        std::string status_message;

        bool isValid() const {
            return pose.isValid() && velocity.isValid() && acceleration.isValid();
        }
    };


    // ============================================================
    // 奇异值分析结果
    // ============================================================

    /**
     * @brief 奇异值分析结果
     */
     struct SingularityAnalysis {
        std::vector<double> singular_values;
        double condition_number; // 条件数
        bool is_near_singular;
        double smallest_singular_value; // 最小奇异值
        Eigen::Vector3d directional_manipulability;
        double manipulability_index; // 最大奇异值

        SingularityAnalysis() : condition_number(0.0), is_near_singular(false), 
                                smallest_singular_value(0.0), manipulability_index(0.0){}
     };


    // ============================================================
    // DH参数结构体
    // ============================================================

    /**
     * @brief DH参数定义
     */
    struct DHParameters {
        double a;       // 连杆长度
        double alpha;   // 连杆扭角
        double d;       // 连杆偏移量
        double theta;   // 关节角度
        bool is_revolute; // true=旋转关节, false=移动关节

        DHParameters() : a(0.0), alpha(0.0), d(0.0), theta(0.0), is_revolute(true){}
        DHParameters(double a_, double alpha_, double d_, double theta_, bool revolute = true) : 
            a(a_), alpha(alpha_), d(d_), theta(theta_), is_revolute(revolute){}

        /**
         * @brief 转换为KDL连杆
         */
        KDL::Segment toKDLSegment(const std::string& name= "DEFAULT_NAME") const{
            KDL::Rotation rotation = KDL::Rotation::RotX(alpha) * KDL::Rotation::RotZ(theta); // Rotation 3×3旋转矩阵，RotX 绕x轴旋转
            KDL::Vector translation(a, -d * std::sin(alpha), d * std::cos(alpha));

            KDL::Frame frame(rotation, translation); // Frame 坐标系

            if(is_revolute) {
                return KDL::Segment(name, KDL::Joint(KDL::Joint::RotZ), frame); // RotZ 绕Z轴旋转的关节
            } else {
                return KDL::Segment(name,KDL::Joint(KDL::Joint::TransZ), frame); // Trans 沿Z轴方向平移的关节
            }
        }
    };


    // ============================================================
    // 机械臂运动学主类
    // ============================================================

    /**
     * @brief 基于KDL和Eigen的机械臂运动学计算类
     * 
     * 该类封装了使用KDL进行正向运动学计算、使用Eigen进行矩阵运算的
     * 完整机械臂运动学解决方案。
     */
    class RobotArmKinematics {
        public:
            /**
             * @brief 构造函数
             * @param num_joints 关节数量
             */
            explicit RobotArmKinematics(size_t num_joints = 6);

            /**
             * @brief 析构函数
             */
            ~RobotArmKinematics();

            // 禁止拷贝
            RobotArmKinematics(const RobotArmKinematics&) = delete;
            RobotArmKinematics& operator = (const RobotArmKinematics&) = delete;
            RobotArmKinematics(RobotArmKinematics&&) = delete;
            RobotArmKinematics& operator=(RobotArmKinematics&&) = delete;


            // ============================================================
            // 初始化方法
            // ============================================================

            /**
             * @brief 使用DH参数初始化机械臂链
             * @param dh_params DH参数列表
             * @return true 初始化成功
             */
            bool initFromDHParams(const std::vector<DHParameters>& dh_params);

            /**
             * @brief 使用标准机械臂参数初始化
             * @return true 初始化成功
             */
            bool initStandard6DOF();

            /**
             * @brief 从URDF文件加再机械臂模型
             * @param urdf_path URDF文件路径
             * @param base_link 基座连接名称
             * @param tip_link 末端链接名称
             * @return true 加载成功
             */
             bool loadFromURDF (const std::string& urdf_path, 
                                const std::string& base_link = "base_link", 
                                const std::string& tip_link = "tool0");

            /**
             * @brief 手动构建机械臂链 
             * @param chain KDL机械臂链
             */
            void setChain(const KDL::Chain& chain);

            /**
             * @brief 设置关节限位
             * @param limits 关节限位配置
             */
            void setJointLimits(const JointLimits& limits);

            /**
             * @brief 配置控制器参数
             * @param config 控制器参数
             */
            void setConfig(const ControllerConfig& config);

            /**
            * @brief 检查是否已初始化
            * @return true 已初始化
            */
            bool isInitialized() const { return is_initialized_.load(); }

            /**
            * @brief 获取关节数量
            * @return 关节数量
            */
            size_t getNumJoints() const { return num_joints_; }


            // ============================================================
            // 正向运动学
            // ============================================================

            /**
             * @brief 计算正向运动学（末端位姿）
             * @param joint_positions 关节位置 [rad 或 m]
             * @return 末端执行器位姿
             * @throws DimensionMismatchException 维度不匹配
             * @throws NotInitializedException 未初始化
             */
            EndEffectorPose computeForwardKinematics(const Eigen::VectorXd& joint_positions);

            /**
             * @brief 计算正向运动学（完整变换矩阵）
             * @param joint_positions 关节位置
             * @return 4x4齐次变换矩阵
             */
            Eigen::Matrix4d computeForwardKinematicsMatrix(const Eigen::VectorXd& joint_position); 


            // ==============================================================
            // 逆向运动学
            // ==============================================================

            /**
             * @brief 数值法逆运动学（位姿）
             * @param target_pose 目标末端位姿
             * @param initial_guess 初始关节角
             * @return 求解得到的关节角
             * 
             * @throws DimensionMismatchException 维度不匹配
             * @throws KinematicSolverException IK求解失败
             * @throws JointLimitException 超出关节限位
             */
            Eigen::VectorXd inverseKinematics(
                const EndEffectorPose& target_pose,
                const Eigen::VectorXd& initial_guess
            );

            /**
             * @brief 带成功标志的逆运动学（不抛异常版本，适合实时）
             * @param target_pose 目标末端位姿
             * @param initial_guess 初始关节角
             * @param solution 输出解
             * @return true 成功
             */
            bool inverseKinematics(
                const EndEffectorPose& target_pose,
                const Eigen::VectorXd& initial_guess,
                Eigen::VectorXd& solution
            );


            // ============================================================
            // 雅可比矩阵
            // ============================================================

            /**
             * @brief 计算雅克比矩阵
             * @param joitn_positions 关节位置
             * @return 6×n 雅克比矩阵
             * @throws DimensionMismatchException 维度不匹配
             */
            Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& joint_positions);

            /**
             * @brief 计算雅克比矩阵的导数
             * @param joint_positions 关节位置
             * @param joint_velocities 关节速度
             * @return 6×n 雅克比矩阵导数
             */
            Eigen::MatrixXd computeJacobianDerivative(
                const Eigen::VectorXd& joint_positions, 
                const Eigen::VectorXd& joint_velocities
            );


            // ============================================================
            // 速度计算
            // ============================================================

            /**
             * @brief 正向速度运动学：从关节速度计算末端速度
             * @param joint_positions 关节位置
             * @param joint_velocities 关节速度
             * @return 末端速度(Twist)
             * @throw DimensionMismatchExpection 维度不匹配
             * @throw SingularityException 奇异位形
             */
            CartesianTwist forwardVelocityKinematics(
                const Eigen::VectorXd& joitn_positions, 
                const Eigen::VectorXd& joint_velocities
            );

            /**
             * @brief 逆向速度运动学：从末端速度计算关节速度
             * @param joint_position 关节位置
             * @param desired_velocity 期望末端速度
             * @return 关节速度
             */
            Eigen::VectorXd inverseVelocityKinematics(
                const Eigen::VectorXd& joint_positions, 
                const CartesianTwist& desired_velocity
            );

            /**
             * @brief 使用阻尼最小二乘法计算逆速度
             * @param joint_positions 关节位置
             * @param desired_velocity 期望末端速度
             * @param damping_factor 阻尼因子
             * @return 关节速度
             */
            Eigen::VectorXd dampedLeastSquaresInverseVelocity(
                const Eigen::VectorXd& joint_positions, 
                const CartesianTwist& desired_velocity, 
                double damping_factor = -1.0
            );


            // ============================================================
            // 加速度计算
            // ============================================================

            /**
             * @brief 正向加速度运动学：从关节加速度计算末端加速度
             * @param joint_positions 关节位置
             * @param joint_velocities 关节速度
             * @param joint_accelerations 关节加速度 [rad/s²]
             * @return 末端加速度
             */
            CartesianAcceleration forwardAccelerationKinematics(
                const Eigen::VectorXd& joint_positions, 
                const Eigen::VectorXd& joint_velocities, 
                const Eigen::VectorXd& joint_accelerations
            );


            // ============================================================
            // 完整运动状态
            // ============================================================

            /**
             * @brief 计算完整的笛卡尔空间运动状态
             * @param joint_state 关节状态（位置、速度、加速度）
             * @return 笛卡尔空间运动状态
             */
            CartesianMotionState computeFullCartesianState(
                const Eigen::VectorXd& joint_positions, 
                const Eigen::VectorXd& joint_velocities, 
                const Eigen::VectorXd& joint_accelerations
            );


            // ============================================================
            // 奇异值分析
            // ============================================================

            /**
             * @brief 分析雅克比矩阵的奇异值
             * @param joint_positions 关节位置
             * @return 奇异值分析结果
             */
            SingularityAnalysis analyzeSingularity(const Eigen::VectorXd& joint_positions);

            /**
             * @brief 检查是否接近奇异位形
             * @param joint_positions 关节位置
             * @return true 接近奇异位形
             */
            bool isNearSingularity(const Eigen::VectorXd& joint_positions);

            /**
             * @brief 计算可操作性指标
             * @param joint_positions 关节位置
             * @return 可操作性指标
             */
            double computeManipulabilityIndex(const Eigen::VectorXd& joint_positions);

            
            // ============================================================
            // 工具方法
            // ============================================================

            /**
             * @brief 验证输入数据的有效性
             * @param data 输入数据
             * @param context 上下文描述
             * @throws DimensionMismatchException 维度不匹配
             */
            template<typename Derived>
            void validateInput(
                const Eigen::MatrixBase<Derived>& data, 
                const std::string& context
            ) const;

            /**
             * @brief 检查数据是否为有限值
             * @param data 输入数据
             * @param context 上下文描述
             * @throws std::runtime_error 包含无效值
             */
            template<typename Derived>
            void checkFinite(
                const Eigen::MatrixBase<Derived>& data,  // MatrixBase 通用矩阵基类
                const std::string& context
            ) const;

            /**
             * @brief 获取最后计算的雅克比矩阵
             * @return 雅克比矩阵
             */
            const Eigen::MatrixXd& getLastJacobian() const{return last_jacobian_;}

            /**
             * @brief 获取状态信息
             * @return 状态消息
             */
            const std::string& getStatusMessage() const {return status_message_;}

        
        private:
            /**
             * @brief 初始化 KDL 运动学求解器
             *
             * 初始化顺序：
             *  1. ChainFkSolverPos_recursive
             *  2. ChainIkSolverVel_pinv
             *  3. ChainIkSolverPos_NR（依赖前两者）
             *
             * 若未正确初始化，IK 求解会直接失败
             */
             void initSolvers();

            /** 
             * @brief 预分配内存
             */
            void preallocateMemory();

             /**
             * @brief 验证关节位置是否在限位范围内
             * @param positions 关节位置
             * @throws JointLimitException 超出限位
             */
            void validateJointLimits(const Eigen::VectorXd& positions) const;

            // KDL组件
            std::unique_ptr<KDL::Chain> chain_;
            std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;  // ChainFkSolverPos_recursive 计算机器人正向运动学
            std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;   // 计算雅克比矩阵
            std::unique_ptr<KDL::ChainJntToJacDotSolver> jacobian_dot_solver_;  // 计算雅克比矩阵的时间导数

            // 内部状态
            size_t num_joints_;
            std::atomic<bool> is_initialized_{false};
            std::atomic<int> error_count_{0};

            // KDL  数据容器（预分配以避免运行时分配）
            mutable KDL::JntArray kdl_joint_positions_;  // mutable 允许在 const 成员函数中修改该变量。
            mutable KDL::JntArray kdl_joint_velocities_;  // KDL::JntArray，KDL 库中的关节数组
            mutable KDL::JntArray kdl_joint_accelerations_;
            mutable KDL::Jacobian kdl_jacobian_;  // KDL::Jacobian 表示雅克比矩阵

            // Eigen数据容器
            Eigen::MatrixXd last_jacobian_;
            Eigen::MatrixXd last_jacobian_dot_; // 缓存上一时刻的雅克比矩阵及其导数

            // 配置
            JointLimits joint_limits_;
            ControllerConfig config_;

            // 状态消息
            std::string status_message_;
            mutable std::mutex status_mutex_;

            // ================= IK 求解器 =================
            //
            // 速度级 IK 求解器（Jacobian 伪逆）
            // 用于：
            //  1. 独立的逆速度运动学
            //  2. 为位姿级 IK（NR 法）提供 dq = J⁺ * dx
            std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;

            // 位姿级 IK 求解器（Newton-Raphson 数值迭代）
            // 依赖：
            //  1. fk_solver_
            //  2. ik_vel_solver_
            // 用于：
            //  给定末端 Frame，迭代求解关节角
            std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;



            static KDL::Frame toKDLFrame(const EndEffectorPose& pose)
            {
                const Eigen::Matrix3d R = pose.orientation.toRotationMatrix();

                return KDL::Frame(
                    KDL::Rotation(
                        R(0,0), R(0,1), R(0,2),
                        R(1,0), R(1,1), R(1,2),
                        R(2,0), R(2,1), R(2,2)
                    ),
                    KDL::Vector(
                        pose.position.x(),
                        pose.position.y(),
                        pose.position.z()
                    )
                );
            }

    };

    // ============================================================
    // 模板方法实现
    // ============================================================

    template<typename Derived>
    void RobotArmKinematics::validateInput(
        const Eigen::MatrixBase<Derived>& data,
        const std::string& context) const {
        
        if (!is_initialized_.load()) {
            throw NotInitializedException("RobotArmKinematics");
        }

        // 输入维度是否与关节数匹配
        if (static_cast<size_t>(data.size()) != num_joints_) {
            throw DimensionMismatchException(
                static_cast<int>(num_joints_), // 转换类型
                static_cast<int>(data.size()),
                context
            );
        }
    }

    // 是否为有效值
    template<typename Derived>
    void RobotArmKinematics::checkFinite(
        const Eigen::MatrixBase<Derived>& data,
        const std::string& context) const {
        
        if (!config_.enable_nan_check) return; // 提供关闭检查选择

        for (int i = 0; i < data.size(); ++i) {
            if (!std::isfinite(data(i))) {
                throw std::runtime_error(
                    "Non-finite value detected in " + context + 
                    " at index " + std::to_string(i)
                );
            }
        }
    }

} // namespace RobotKinematicsKDL

#endif // ROBOT_KINEMATICS_KDL_HPP






