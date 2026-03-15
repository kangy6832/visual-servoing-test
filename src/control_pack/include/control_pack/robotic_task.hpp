#pragma once 

#include "robot_pose_polynomial.hpp"
#include "velocity_ik_generator.hpp"

#include "control_pack/robot_pose_polynomial.hpp"
#include "control_pack/kdl_dynamics.hpp"
#include "visualization_msgs/msg/marker.hpp" 
#include <geometry_msgs/msg/detail/pose__struct.hpp> 
#include <geometry_msgs/msg/pose.hpp> 
#include <geometry_msgs/msg/transform_stamped.hpp> 
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp> 
#include <rclcpp/parameter_client.hpp> 
#include <rclcpp/publisher.hpp> 
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2_ros/buffer.h> 
#include <tf2_ros/transform_listener.hpp> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <geometry_msgs/msg/pose.hpp> 
#include <sensor_msgs/msg/joint_state.hpp> 
#include <robot_interfaces/msg/robot.hpp> 
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h> 
#include <moveit_msgs/msg/collision_object.h> 
#include <moveit/utils/moveit_error_code.h> 
#include <rclcpp/rclcpp.hpp> 
#include <rclcpp_action/server.hpp> 
#include <rclcpp_action/server_goal_handle.hpp> 
#include <shape_msgs/msg/solid_primitive.hpp> 
#include <rclcpp_action/create_client.hpp> 
#include <robot_interfaces/action/catch.hpp> 
#include <thread> 
#include <memory> 
#include <atomic> 
#include <Eigen/Dense> 
#include <Eigen/Geometry> 
#include <vector>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <Eigen/Dense>

typedef enum{
    ROBOTIC_ARM_TASK_MOVE = 1,          // 移动任务
    ROBOTIC_ARM_TASK_CATCH_TARGET = 2,  // 捕获任务
    ROBOTIC_ARM_TASK_PLACE_TARGET = 3   // 放置任务
}ArmTask;

/**
 * @brief 任务类型
 */
typedef enum{
    ROBOTIC_ARM_TASK_STATE_IDLE = 1, // 初始空闲状态
    ROBOTIC_ARM_TASK_STATE_MOVE_TO_READY_CATCH_POINT = 2, // 移动到预备抓取位置
    ROBOTIC_ARM_TASK_STATE_MOVE_TO_CATCH_POINT = 3, // 移动到抓取位置
    ROBOTIC_ARM_TASK_STATE_CATCH_TARGET = 4, // 抓取目标
    ROBOTIC_ARM_TASK_STATE_VISUAL_SERVOING = 5, // 视觉伺服
    ROBOTIC_ARM_TASK_STATE_MOVE_TO_RELEASE_POINT = 6, // 移动到释放位置
    ROBOTIC_ARM_TASK_STATE_RELEASE_TARGET = 7, // 释放目标
    ROBOTIC_ARM_TASK_STATE_MOVE_TO_IDLE_POINT = 8, // 移动到空闲位置
    ROBOTIC_ARM_TASK_STATE_MOVE_TO_READY_CATCH_POINT_KFS_NOT_ZERO = 9, // 移动到预备抓取位置（KFS = 1）
    ROBOTIC_ARM_TASK_STATE_MOVE_TO_RELEASE_POINT_IN_SHELF = 10 // 移动到释放位置（将kfs放到架子上）
}ArmTaskState;

namespace robotic_task {
    class RoboticTask{
        public:
            explicit RoboticTask(const rclcpp::Node::SharedPtr node);
            ~RoboticTask();

            enum class Exercise {
                POSITION, 
                VELOCITY
            };
        
        private:

            // 接近模式枚举：定义接近目标的方式（自动或指定位置模式）
            enum class ApproachMode {
                AUTO, 
                POS
            };

            // 操作成功标志
            bool success;
            // ROS2节点共享指针，用于通信和资源管理
            rclcpp::Node::SharedPtr node;
            
            // 任务互斥锁，用于线程安全访问任务状态
            std::mutex task_mutex_;
            // 任务条件变量，用于线程间任务通知
            std::condition_variable task_cv_;
            // 新任务标志，表示是否有新的任务需要处理
            bool has_new_task_{false};

            // 异步参数客户端，用于与驱动节点通信获取/设置参数
            rclcpp::AsyncParametersClient::SharedPtr param_client;
            // 当前动作目标句柄，用于管理抓取动作的执行
            std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> current_goal_handle;
            // 机器人模型常量指针，用于运动学计算
            moveit::core::RobotModelConstPtr robot_module;
            // 机械臂任务执行线程
            std::unique_ptr<std::thread> arm_task_thread;

            // 任务目标位置和姿态
            geometry_msgs::msg::Pose task_target_pos;
            
            // 机械臂任务运行标志（原子变量，线程安全）
            std::atomic<bool> is_running_arm_task{false};

            // 当前任务类型（原子变量：0=无任务, 1=移动, 2=抓取, 3=放置）
            std::atomic<int> current_task_type{0}; // 任务类型
            
            // 当前KFS编号（运动反馈系统编号）
            std::atomic<int> current_kfs_num{0};

            // 取消当前任务标志
            std::atomic<bool> cancle_current_task{false};
            // 当前状态（原子变量，使用ArmTaskState枚举）
            std::atomic<int> current_state{ArmTaskState::ROBOTIC_ARM_TASK_STATE_IDLE}; // 当前状态
            
            
            // 相机到link0的坐标变换缓冲区
            std::unique_ptr<tf2_ros::Buffer> camera_link0_tf_buffer;
            // 相机到link0的坐标变换监听器
            std::shared_ptr<tf2_ros::TransformListener> camera_link0_tf_lisenter_;
            // 相机到link0的坐标变换数据
            geometry_msgs::msg::TransformStamped camera_link0_tf;
            
            // 相机到link6的坐标变换缓冲区
            std::unique_ptr<tf2_ros::Buffer> camera_link6_tf_buffer;

            // 相机到link6的坐标变换监听器
            std::shared_ptr<tf2_ros::TransformListener> camera_link6_tf_lisenter_;

            // 相机到link6的坐标变换数据
            geometry_msgs::msg::TransformStamped camera_link6_tf;

            // link4到link6的坐标变换缓冲区
            std::unique_ptr<tf2_ros::Buffer> link4_link6_tf_buffer;
            // 通用坐标变换缓冲区
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            // 通用坐标变换监听器
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            

            // 目标物体坐标（相机为原点）
            Eigen::Vector3d target_object_position;


            // MoveIt运动规划接口，用于规划和执行机械臂运动
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;

            // 可视化标记发布器，用于在RViz中显示目标位置等
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mark_pub_;

            // 附加的KFS位置（相对于link6，用于碰撞检测）
            geometry_msgs::msg::Pose attached_kfs_pos;

            // 规划场景接口，用于添加/删除碰撞对象等场景管理
            std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi; // planning_scene_interface,用于操作场景,包括添加、删除、更新等

            // 处理动作目标的响应函数
            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID& uuid, 
                std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal
            );
            // 处理动作取消的响应函数
            rclcpp_action::CancelResponse cancel_goal(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>>& goal_handle
            );
            // 处理接受的动作目标的函数
            void handle_accepted(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>>& goal_handle
            );

            int count ; // 用于循环重试的数量表示
            const int MAX_COUNT = 100; // 表示最大重试次数


            const double VELOCITY_SCALING = 0.4; // 最大速度缩放因子
            const double ACCELERATION_SCALING = 0.3; // 最大加速度缩放因子
            const double MAX_END_EFFECTOR_VELOCITY = 1.0; // 最大末端执行器速度 [m/s]
            const double MAX_END_EFFECTOR_ACCELERATION = 1.0; // 最大末端执行器加速度 [m/s^2]

            void arm_catch_task_handle();


            /**
             * @brief 向机器人添加附加的KFS碰撞对象
             * 
             * 创建并附加一个代表KFS（运动反馈系统）的碰撞对象
             * 到机器人的link6，用于模拟抓取后携带的物体。
             * 
             * @return 操作成功返回true，失败返回false
             */
            bool add_attached_kfs_collision();


            /**
             * @brief 从机器人移除附加的KFS碰撞对象
             * 
             * 从link6移除附加的KFS碰撞对象，并清理规划场景中
             * 对应的独立碰撞对象。通常在释放物体后调用。
             * 
             * @return 操作成功返回true，失败返回false
             */
            bool remove_attached_kfs_collision();


            /**
             * @brief 向规划场景添加KFS碰撞对象
             * 
             * 在规划场景中添加一个独立的KFS碰撞对象，用于表示
             * 环境中需要避障或交互的物体。
             * 
             * @param pos 碰撞对象的位置和姿态
             * @param object_id 对象的唯一标识符
             * @param frame_id 对象位置的参考坐标系
             * @return 操作成功返回true，失败返回false
             */
            bool add_kfs_collision(
                const geometry_msgs::msg::Pose& pos, 
                const std::string& object_id, 
                const std::string& frame_id
            );


            /**
             * @brief 从规划场景移除KFS碰撞对象
             * 
             * 使用对象ID从规划场景中移除指定的碰撞对象。
             * 用于清理不再需要避障的物体。
             * 
             * @param object_id 要移除对象的唯一标识符
             * @param frame_id 对象的参考坐标系
             * @return 操作成功返回true，失败返回false
             */
            bool remove_kfs_collision(
                const std::string& object_id, 
                const std::string& frame_id
            );


            /**
             * @brief 设置气泵启用/禁用状态
             * 
             * 通过ROS2参数服务控制气泵（吸盘）的开关状态。
             * 包含重试机制以确保操作的可靠性。
             * 
             * @param enable true启用气泵，false禁用气泵
             * @return 操作成功返回true，失败返回false
             */
            bool set_air_pump(bool enable);


            /**
            * @brief 验证气泵状态是否匹配期望值
            * 
            * 通过读取驱动节点参数检查气泵的实际状态，
            * 确保气泵正确响应了控制命令。
            * 
            * @param expected_status 期望的气泵状态
            * @return 状态匹配返回true，不匹配返回false
            */
            bool verify_air_pump_status(bool expected_status);


            /**
            * @brief 验证抓取操作是否成功
            * 
            * 通过检查关节电流、真空传感器等指标验证抓取成功。
            * 当前实现基于简单延迟，可扩展为更复杂的验证逻辑。
            * 
            * @return 抓取成功返回true，失败返回false
            */
            bool verify_grasp_success();



            /**
            * @brief 计算接近和抓取的目标位姿
            * 
            * 基于物体位置和方向计算最优的接近和抓取位姿。
            * 考虑机器人运动学约束和碰撞避免，计算末端执行器的
            * 最佳姿态和路径。
            * 
            * @param box_pos 目标物体的位置和方向
            * @param approach_distance 接近位姿距离抓取位置的距离
            * @param grasp_pose 输出参数，返回计算的抓取位姿
            * @param mode 接近模式（AUTO或POS）
            * @return 计算的接近位姿
            */
            geometry_msgs::msg::Pose calculate_target_pose(
                const geometry_msgs::msg::Pose& box_pos, 
                double approach_distance, 
                geometry_msgs::msg::Pose& grasp_pose, 
                int mode 
            );


            /**
            * @brief 计算准备位姿（使用固定方向）
            * 
            * 使用固定方向计算简化的准备位姿。适用于测试或
            * 不需要复杂方向计算的场景。
            * 
            * @param box_pos 目标物体的位置和方向
            * @param approach_distance 接近位姿距离抓取位置的距离
            * @param grasp_pose 输出参数，返回计算的抓取位姿
            * @param mode 接近模式（当前未使用）
            * @return 计算的准备位姿
            */
            geometry_msgs::msg::Pose calculate_prepare_pose_with_orientation(
                const geometry_msgs::msg::Pose& box_pos, 
                double approach_distance, 
                geometry_msgs::msg::Pose &grasp_pose, 
                int mode
            );

            
            

            // ======================================= 头文件接口 =======================================
            // 五次多项式接口
            RobotPosePolynomial::QuinticParam robot_pose_polynomial_QuinticParam;
            RobotPosePolynomial::ContinuousTrajectory robot_pose_polynomial_ContinuousTrajectory;

            //  六维空间速度向量（Twist）
            RobotKinematicsKDL::CartesianTwist velocity_ik_generator_CartesianTwist;
            // 基于KDL和Eigen的机械臂运动学计算类
            RobotKinematicsKDL::RobotArmKinematics velocity_ik_generator_RobotArmKinematics;

            // KDL动力学计算类
            std::unique_ptr<robotic_task::KDLDynamics> kdl_dynamics_;

            // 动力学控制参数
            struct DynamicsControlParams {
                double control_frequency = 50.0;           // Hz
                double end_effector_acceleration = 0.1;    // m/s²
                double max_joint_acceleration = 1.0;       // rad/s²
                double compensation_gain = 0.1;            // 动力学补偿增益
                double max_joint_torque = 50.0;            // N·m 关节力矩限制
                double emergency_stop_threshold = 100.0;   // 紧急停止阈值
                bool enable_dynamics_compensation = true;   // 启用动力学补偿
                bool enable_acceleration_control = true;    // 启用加速度控制
                double trajectory_kp = 100.0;               // 轨迹跟踪比例增益
                double kfs_payload_mass = 0.50;             // kg 抓取KFS后的附载质量
            } dynamics_params_;

            // 抓取后是否存在附载（用于附加载荷重力补偿）
            bool has_attached_kfs_{false};
            // KFS质心在末端执行器坐标系中的偏移（m）
            Eigen::Vector3d kfs_payload_com_in_ee_{0.0, 0.0, -0.24};

            // 末端速度
            Eigen::Vector3d end_effector_velocity;
            

            // ROS2 的末端速度表示
            geometry_msgs::msg::Twist ros2_end_effector_velocity;

            // ROS2 的末端速度发布器
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ros2_end_effector_velocity_pub;

            // ROS2 的末端速度订阅器
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ros2_end_effector_velocity_sub;

            /**
             * @brief 关节位置（关节角）
             * 
             * 最开始，关节位置为零，即各关节的角度为0°。
             * 后，使用 get_joint_position() 获取上次规划后的关节位置，并作为当前的关节位置。
             * 这样，在规划过程中，关节位置会不断更新，直到达到目标位置。
             */
            Eigen::VectorXd joint_position;
            Eigen::VectorXd temp_joint_position; // 临时关节位置


            

            /**
            * @brief 根据目标坐标构建末端速度向量
            * 
            * 根据输入的XYZ坐标值创建末端执行器的速度向量。
            * 该函数用于将目标坐标转换为速度控制所需的向量格式。
            * 
            * @param x X方向的速度分量
            * @param y Y方向的速度分量
            * @param z Z方向的速度分量
            */
            void calculate_end_effector_velocity(
                double x, 
                double y, 
                double z
            );


            /**
            * @brief 根据末端速度计算关节速度
            * 
            * 使用机器人运动学逆解，将末端执行器的笛卡尔速度
            * 转换为各关节的角速度。这是速度控制的核心函数。
            * 
            * @param end_effector_velocity 末端执行器的3D速度向量
            * @return 各关节的角速度向量
            */
            Eigen::VectorXd calculate_joint_velocity(
                const Eigen::Vector3d& end_effector_velocity
            ) const;


            /**
            * @brief 根据末端速度计算关节加速度
            * 
            * 基于末端速度和当前关节速度，计算各关节所需的加速度。
            * 该函数用于实现平滑的速度控制和轨迹跟踪。
            * 
            * @param end_effector_velocity 末端执行器的3D速度向量
            * @param joint_velocity 当前各关节的角速度向量
            * @return 各关节的加速度向量
            */
            Eigen::VectorXd calculate_joint_acceleration(
                const Eigen::Vector3d& end_effector_velocity, 
                const Eigen::VectorXd& joint_velocity
            ) const;

            /**
            * @brief 发送关节速度命令到硬件驱动节点
            * 
            * 将计算出的关节速度通过ROS2话题发布给硬件驱动节点，
            * 实现对机械臂的速度控制。
            * 
            * @param joint_velocities 要发送的关节速度向量
            * @return 发送成功返回true，失败返回false
            */
            bool send_joint_velocity_to_hardware(const Eigen::VectorXd& joint_velocities);

            /**
            * @brief 发送关节力矩命令到硬件驱动节点
            * 
            * 将计算出的关节力矩通过ROS2话题发布给硬件驱动节点，
            * 实现对机械臂的力矩控制。
            * 
            * @param joint_torques 要发送的关节力矩向量
            * @return 发送成功返回true，失败返回false
            */
            bool send_joint_torque_to_hardware(const Eigen::VectorXd& joint_torques);


            


            /**
            * @brief 获取当前关节位置
            * 
            * 返回机器人当前各关节的角度位置。该位置由关节状态
            * 订阅器实时更新，用于运动规划和控制。
            * 
            * @return 当前关节位置的向量
            */
            Eigen::VectorXd get_joint_position() const;

            /**
            * @brief 检查关节力矩是否在安全范围内
            * 
            * @param joint_torques 关节力矩向量
            * @return true 安全，false 超出限制
            */
            bool checkJointTorqueLimits(const Eigen::VectorXd& joint_torques) const;

            /**
            * @brief 检查是否需要紧急停止
            * 
            * @param joint_positions 关节位置
            * @param joint_velocities 关节速度
            * @param joint_accelerations 关节加速度
            * @return true 需要紧急停止
            */
            bool checkEmergencyStop(
                const Eigen::VectorXd& joint_positions,
                const Eigen::VectorXd& joint_velocities,
                const Eigen::VectorXd& joint_accelerations
            ) const;

            /**
            * @brief 计算抓取KFS后的附加载荷重力补偿
            *
            * @param joint_positions 当前关节位置
            * @return 由KFS附载引入的关节力矩补偿
            */
            Eigen::VectorXd calculate_kfs_payload_compensation(
                const Eigen::VectorXd& joint_positions
            ) const;

            /**
            * @brief 更新动力学控制参数
            * 
            * @param params 新的控制参数
            */
            void updateDynamicsParams(const DynamicsControlParams& params);

            // 实现关节位置的实时更新
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

            // 关节速度发布器，用于向硬件驱动节点发送速度命令
            rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr joint_velocity_publisher_;

            /**
            * @brief 关节状态更新的回调函数
            * 
            * 当接收到新的关节状态数据时自动调用。该函数更新
            * joint_position向量，确保运动规划使用最新的关节位置。
            * 
            * @param msg 关节状态消息的共享指针，包含各关节的当前角度
            */
            void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);


            
            
            
            /**
            * @brief ROS2末端速度回调函数
            * 
            * 处理来自其他节点的末端速度控制命令。当接收到速度
            * 消息时，将其转换为关节控制指令并执行。
            * 
            * @param msg 末端速度消息的共享指针，包含线速度和角速度
            */
            void ros2EndEffectorVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);


            rclcpp_action::Server<robot_interfaces::action::Catch>::SharedPtr arm_handle_server; 


            // 状态机相关函数

            /**
            * @brief 执行移动任务
            * 
            * 执行简单的移动任务，包括移动到预备位置和抓取位置。
            * 适用于只需要定位而不需要抓取的场景。
            * 
            * @return 任务成功返回true，失败返回false
            */
            bool execute_move_task();


            /**
            * @brief 执行抓取任务
            * 
            * 执行完整的抓取序列：移动到预备位置→抓取位置→抓取目标
            * →移动到释放位置→释放目标→返回空闲位置。
            * 
            * @return 任务成功返回true，失败返回false
            */
            bool execute_catch_task();



            /**
            * @brief 执行放置任务
            * 
            * 执行与抓取任务相同的状态序列，但释放逻辑不同。
            * 适用于将物体放置到指定位置的场景。
            * 
            * @return 任务成功返回true，失败返回false
            */
            bool execute_place_task();

            
            /**
            * @brief 处理初始空闲状态
            * 
            * 管理机器人等待新任务的空闲状态。更新反馈信息
            * 并执行最小处理，保持系统响应性。
            * 
            * @return 状态处理成功返回true，失败返回false
            */
            bool handle_idle_state();


            /**
            * @brief 处理移动到预备抓取点状态
            * 
            * 将机器人移动到预备抓取位置，为实际抓取操作
            * 提供良好的可见性和接近角度。
            * 
            * @return 状态处理成功返回true，失败返回false
            */
            bool handle_move_to_ready_catch_point();


            /**
            * @brief 处理移动到抓取点状态
            * 
            * 将机器人从预备位置精确移动到抓取位置。
            * 可能需要使用视觉伺服进行最终定位。
            * 
            * @return 状态处理成功返回true，失败返回false
            */
            bool handle_move_to_catch_point();
            bool handle_move_to_catch_point_kfs_not_zero();


            /**
            * @brief 处理抓取目标状态
            * 
            * 执行实际抓取操作：添加碰撞对象、激活气泵、
            * 验证抓取成功、添加附加碰撞对象。
            * 
            * @return 状态处理成功返回true，失败返回false
            */
            bool handle_catch_target();


            /**
            * @brief 处理移动到释放点状态
            * 
            * 将携带物体的机器人移动到释放位置。
            * 根据任务类型计算不同的释放点。
            * 
            * @return 状态处理成功返回true，失败返回false
            */
            bool handle_move_to_release_point();


            /**
            * @brief 处理释放目标状态
            * 
            * 执行释放操作：停用气泵、等待释放完成、
            * 移除碰撞对象。
            * 
            * @return 状态处理成功返回true，失败返回false
            */
            bool handle_release_target();


            /**
            * @brief 处理移动到空闲点状态
            * 
            * 将机器人移动到安全的空闲位置，通常为初始位置
            * 或预设的静止位置。
            * 
            * @return 状态处理成功返回true，失败返回false
            */
            bool handle_move_to_idle_point();

            
            /**
            * @brief 转换到新状态
            * 
            * 更新当前状态并记录转换信息，用于状态机管理
            * 和调试跟踪。
            * 
            * @param new_state 要转换到的新状态
            */
            void transition_to_state(ArmTaskState new_state);


            /**
            * @brief 更新动作反馈信息
            * 
            * 向动作客户端发送当前状态和描述信息，
            * 保持客户端对任务进度的了解。
            */
            void update_feedback();


            /**
            * @brief 重置任务状态
            * 
            * 将所有任务相关变量和标志重置为初始状态，
            * 为下一个任务执行做准备。
            */
            void reset_task_state();


            /**
            * @brief 获取状态的描述字符串
            * 
            * 将状态枚举值转换为人类可读的字符串，
            * 用于日志记录和调试显示。
            * 
            * @param state 要获取描述的状态枚举值
            * @return 状态的中文描述字符串
            */
            std::string get_state_description(ArmTaskState state); 
    };

}  // namespace robotic_task
