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
     class KinematicSolverExpection : public std::runtime_error{
        public:
            explicit KinematicSolverExpection(const std::string& msg) : std::runtime_error(msg){}
            KinematicSolverExpection(int error_code, const std::string& solver_name) : 
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
     class NotInitialzedExpection : public std::runtime_error{
        public :
            explicit NotInitialzedExpection(const std::string& component) : std::runtime_error(component + 
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
                Eigen::Vector3d(a[4], a[5], a[6]));
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
            T.block<3, 3>(0, 0) = orientation.toRotationMatrix();
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
        double condition_number;
        bool is_near_singular;
        double smallest_singular_value;
        Eigen::Vector3d directional_manipulability;
        double manipulability_index;

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
            KDL::Rotation rotation = KDL::Rotation::RotX(alpha) * KDL::Rotation::RotZ(theta);
            KDL::Vector translation(a, -d * std::sin(alpha), d * std::cos(alpha));
            KDL::Frame frame(rotation, translation);

            if(is_revolute) {
                return KDL::Segment(name, KDL::Joint(KDL::Joint::RotZ), frame);
            } else {
                return KDL::Segment(name,KDL::Joint(KDL::Joint::TransZ), frame);
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
     }

};
