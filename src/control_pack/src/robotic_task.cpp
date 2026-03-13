/**
 * @file robotic_task.cpp
 * @brief 视觉伺服系统RoboticTask类的实现。
 * 
 * 本文件包含RoboticTask类的实现，该类管理机械臂操作，包括移动、抓取和放置任务。
 * 它实现了用于任务执行的状态机，并提供运动规划、碰撞处理和气泵控制的接口。
 * 
 * @author 视觉伺服团队
 * @date 2025
 */

#include "control_pack/robotic_task.hpp"
#include "control_pack/robot_pose_polynomial.hpp"
#include "control_pack/velocity_ik_generator.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cassert>
#include <cstddef>
#include <exception>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/detail/collision_object__struct.hpp>
#include <moveit_msgs/msg/detail/constraints__struct.hpp>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/utilities.hpp>
#include <robot_interfaces/action/detail/catch__struct.hpp>
#include <shape_msgs/msg/detail/solid_primitive__struct.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <rclcpp/parameter_client.hpp>
#include <mutex>
#include <atomic>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <condition_variable>

using namespace std::chrono_literals;
using namespace robotic_task;

/**
 * @brief RoboticTask类的构造函数。
 * 
 * 初始化所有必要的组件，包括：
 * - 关节位置为零向量
 * - 任务管理的动作服务器
 * - 坐标变换的TF2缓冲区和监听器
 * - 运动规划的MoveIt接口
 * - ROS2发布器和订阅器
 * - 碰撞对象和规划场景
 * - 任务执行线程
 * 
 * @param node 用于通信的ROS2节点共享指针
 */
RoboticTask::RoboticTask(const rclcpp::Node::SharedPtr node) : node(node){
    // 初始化关节位置为6维零向量（假设6关节机器人）
    joint_position = Eigen::VectorXd::Zero(6);
    
    // 初始化用于驱动节点通信的异步参数客户端
    param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "driver_node");
    // 创建机器人任务管理的动作服务器
    arm_handle_server = rclcpp_action::create_server<robot_interfaces::action::Catch>(node, "robotic_task_", 
        std::bind(&RoboticTask::handle_goal, this, std::placeholders::_1, std::placeholders::_2), 
        std::bind(&RoboticTask::cancel_goal, this, std::placeholders::_1), 
        std::bind(&RoboticTask::handle_accepted, this, std::placeholders::_1)
    );

    // 初始化坐标框架变换的TF2缓冲区和监听器
    camera_link0_tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    camera_link0_tf_lisenter_ = std::make_shared<tf2_ros::TransformListener>(*camera_link0_tf_buffer);
    camera_link6_tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    camera_link6_tf_lisenter_ = std::make_shared<tf2_ros::TransformListener>(*camera_link6_tf_buffer);


    
    // 初始化运动规划和场景管理的MoveIt接口
    // 加载运动学配置参数
    node->declare_parameter("robot_description_kinematics.robotic_arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    node->declare_parameter("robot_description_kinematics.robotic_arm.kinematics_solver_search_resolution", 0.005);
    node->declare_parameter("robot_description_kinematics.robotic_arm.kinematics_solver_timeout", 0.05);
    node->declare_parameter("robot_description_kinematics.robotic_arm.kinematics_solver_attempts", 10);
    
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robotic_arm");
    psi = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    mark_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("debug_marker", 10);

    // 创建实时关节位置监控的关节状态订阅器
    joint_state_subscriber_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&RoboticTask::jointStateCallback, this, std::placeholders::_1)
    );

    // 创建关节速度命令发布器，用于向硬件驱动节点发送速度控制指令
    joint_velocity_publisher_ = node->create_publisher<robot_interfaces::msg::Robot>(
        "joint_velocity_commands", 10
    );

    // 创建可视化标记发布定时器（显示目标位置）
    node->create_wall_timer
    (
        100ms, 
        [this](){
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            if(!is_running_arm_task) return;
        }

        // 创建和配置目标位置的可视化标记
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->node->now();
        marker.ns = "kfs_pos";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;  // 显示球体标记
        marker.action = visualization_msgs::msg::Marker::ADD;   // 添加标记
        marker.pose = task_target_pos;

        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;  // 设置透明度

        marker.lifetime.sec = 1;  // 标记持续时间
        marker.lifetime.nanosec = 0;  // 标记持续时间

        mark_pub_->publish(marker);
    });  

    // 初始化附加的KFS位置（相对于link6）
    attached_kfs_pos.orientation.w = 1.0;
    attached_kfs_pos.position.x = 0.0;
    attached_kfs_pos.position.y = 0.0;
    attached_kfs_pos.position.z = -0.24;

    // 配置MoveIt运动规划参数
    move_group_interface->setPlanningTime(10.0);
    move_group_interface->setMaxVelocityScalingFactor(VELOCITY_SCALING);            // 设置速度缩放因子
    move_group_interface->setMaxAccelerationScalingFactor(ACCELERATION_SCALING);    // 设置加速度缩放因子
    move_group_interface->setPoseReferenceFrame("base_link");
    move_group_interface->setEndEffectorLink("link6");
    move_group_interface->setGoalPositionTolerance(0.01);
    move_group_interface->setGoalOrientationTolerance(0.05);

    // 初始化KDL动力学计算类
    try {
        kdl_dynamics_ = std::make_unique<robotic_task::KDLDynamics>();
        std::string urdf_path = "/home/kyy/cpp_project/visual_servoing/src/robotic_arm/urdf/robotic_arm.urdf";
        if (kdl_dynamics_->initFromURDF(urdf_path, "world", "link6")) {
            RCLCPP_INFO(node->get_logger(), "KDL动力学初始化成功");
        } else {
            RCLCPP_ERROR(node->get_logger(), "KDL动力学初始化失败");
            kdl_dynamics_.reset();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "KDL动力学初始化异常: %s", e.what());
        kdl_dynamics_.reset();
    }

    // move_group_interface->allowReplanning(true); // 允许重新规划
    // move_group_interface->setPlannerId("RRTConnectkConfigDefault"); // 设置规划器
    // move_group_interface->setNumPlanningAttempts(10); // 设置规划尝试次数

    // move_group_interface->setGoalPositionTolerance(0.01); // 设置目标位置容差
    // move_group_interface->setGoalOrientationTolerance(0.01); // 设置目标方向容差
    // move_group_interface->setPoseReferenceFrame("base_link"); // 设置目标参考坐标系
    // move_group_interface->setEndEffectorLink("wrist_3_link"); // 设置末端效果器链接
    // move_group_interface->setSupportSurfaceName("table"); // 设置支持表面名称

    // 初始化通用坐标变换的主TF2缓冲区和监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());  // 创建TF2缓冲区
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // 创建TF2监听器

    // 创建并启动机械臂任务处理线程
    try{
        arm_task_thread = std::make_unique<std::thread>([this](){arm_catch_task_handle();});
    } catch (const std::exception& e){
        RCLCPP_ERROR(node->get_logger(), "创建机械臂任务线程失败: %s", e.what());
        arm_task_thread = nullptr;
    }
}


/**
 * @brief RoboticTask类的析构函数。
 * 
 * 确保适当的清理，包括：
 * - 通知任务线程退出
 * - 等待线程完成
 * - 释放所有资源
 */
RoboticTask::~RoboticTask(){
    task_mutex_.lock();  // 获取锁
    has_new_task_ = true;
    task_mutex_.unlock();
    task_cv_.notify_one(); 
    
    // 等待任务线程完成
    if(arm_task_thread && arm_task_thread->joinable()){
        arm_task_thread->join();
    }
}


/**
 * @brief 处理传入的动作目标。
 * 
 * 通过以下方式验证目标请求：
 * - 检查是否有其他任务正在运行
 * - 验证相机变换的可用性
 * - 将目标位姿转换到base_link坐标系
 * - 归一化四元数
 * - 设置当前任务类型
 * 
 * @param uuid 目标请求的UUID
 * @param goal 目标消息的共享指针
 * @return 表示接受或拒绝的GoalResponse
 */
rclcpp_action::GoalResponse RoboticTask::handle_goal(
    const rclcpp_action::GoalUUID& uuid, 
    std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal
){
    (void)uuid;
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if(is_running_arm_task){
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // 验证相机到base_link的变换是否可用
    try{
        camera_link0_tf = camera_link0_tf_buffer->lookupTransform("base_link", "camera_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex){
        RCLCPP_WARN(node->get_logger(), "无法获取相机到基座的变换: %s", ex.what());
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 将目标位姿从相机坐标系转换到base_link坐标系
    tf2::doTransform(goal->target_pose, task_target_pos, camera_link0_tf);
    RCLCPP_INFO(node->get_logger(), "原始目标位姿: Pos(%lf, %lf, %lf), Ori(%lf, %lf, %lf, %lf)",
                goal->target_pose.position.x, goal->target_pose.position.y, 
                goal->target_pose.position.z, goal->target_pose.orientation.x, 
                goal->target_pose.orientation.y, goal->target_pose.orientation.z, 
                goal->target_pose.orientation.w);

    // 归一化四元数以确保有效的旋转
    auto qin = task_target_pos.orientation;
    tf2::Quaternion q(qin.x, qin.y, qin.z, qin.w);
    q.normalize();
    task_target_pos.orientation.x = q.x();
    task_target_pos.orientation.y = q.y();
    task_target_pos.orientation.z = q.z();
    task_target_pos.orientation.w = q.w();

    target_object_position << task_target_pos.position.x, task_target_pos.position.y, task_target_pos.position.z;

    current_task_type = goal->action_type; // 设置当前任务类型
    
    // 通知任务线程有新任务
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        has_new_task_ = true;
    }
    task_cv_.notify_one();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


/**
 * @brief 处理动作目标取消请求。
 * 
 * 通过以下方式处理取消：
 * - 设置取消标志
 * - 停止当前任务执行
 * - 移除碰撞对象
 * - 关闭气泵
 * 
 * @param goal_handle 要取消的目标句柄的共享指针
 * @return 表示接受取消的CancelResponse
 */
rclcpp_action::CancelResponse RoboticTask::cancel_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>>& goal_handle 
){
    (void)goal_handle;
    cancle_current_task = true;
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        is_running_arm_task = false;
    }
    
    // 通知任务线程停止等待
    task_cv_.notify_one();

    // 清理碰撞对象和气泵
    remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame());
    set_air_pump(false);
    has_attached_kfs_ = false;

    return rclcpp_action::CancelResponse::ACCEPT;
}


/**
 * @brief 处理已接受的动作目标并开始执行。
 * 
 * 当目标被接受且应开始执行时调用。
 * 设置目标句柄，将任务标记为正在运行，并通知
 * 任务线程开始处理。
 * 
 * @param goal_handle 已接受的目标句柄的共享指针
 */
void robotic_task::RoboticTask::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>>& goal_handle
){
    current_goal_handle = goal_handle;
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        is_running_arm_task = true;
    }
    cancle_current_task = false;

    // 通知任务线程有新任务可用
    task_mutex_.lock();
    has_new_task_ = true;
    task_mutex_.unlock();
    task_cv_.notify_one();
}


/**
 * @brief 主要的机械臂任务处理线程函数。
 * 
 * 此函数在单独的线程中运行，使用状态机方法管理机械臂任务的执行。它：
 * - 为环境设置碰撞对象
 * - 等待任务通知
 * - 根据任务类型执行适当的任务
 * - 通过动作服务器发送回结果
 * - 处理任务状态转换
 */
void robotic_task::RoboticTask::arm_catch_task_handle(){
    RCLCPP_INFO(node->get_logger(), "进入机械臂任务处理线程");
    bool first_run = true;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool continue_flag = false;

    auto feedback_msg = std::make_shared<robot_interfaces::action::Catch::Feedback>();
    auto finish_msg = std::make_shared<robot_interfaces::action::Catch::Result>();

    // 等待系统初始化
    std::this_thread::sleep_for(5s);
    {
        // 为规划场景添加提升机构障碍物
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_interface->getPlanningFrame();
        collision_object.id = "instituion";
        collision_object.primitives.resize(4);
        collision_object.primitive_poses.resize(4);

        // 为所有障碍物设置方向（单位四元数）
        collision_object.primitive_poses[0].orientation.w = 1.0;
        collision_object.primitive_poses[1].orientation.w = 1.0;
        collision_object.primitive_poses[2].orientation.w = 1.0;
        collision_object.primitive_poses[3].orientation.w = 1.0;

        // 在工作空间周围定位障碍物
        collision_object.primitive_poses[0].position.x = -0.60;
        collision_object.primitive_poses[0].position.y = 0.35;
        collision_object.primitive_poses[0].position.z = 0.3;

        collision_object.primitive_poses[1].position.x = 0.05;
        collision_object.primitive_poses[1].position.y = 0.35;
        collision_object.primitive_poses[1].position.z = 0.3;

        collision_object.primitive_poses[2].position.x = -0.60;
        collision_object.primitive_poses[2].position.y = -0.35;
        collision_object.primitive_poses[2].position.z = 0.3;

        collision_object.primitive_poses[3].position.x = 0.05;
        collision_object.primitive_poses[3].position.y = -0.35;
        collision_object.primitive_poses[3].position.z = 0.3;

        // 为障碍物定义盒子形状
        shape_msgs::msg::SolidPrimitive primitive;
        collision_object.primitives[0].type = primitive.BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[primitive.BOX_X] = 0.06;
        collision_object.primitives[0].dimensions[primitive.BOX_Y] = 0.06;
        collision_object.primitives[0].dimensions[primitive.BOX_Z] = 0.6;
        collision_object.primitives[3] = collision_object.primitives[2] = 
        collision_object.primitives[1] = collision_object.primitives[0];
        
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        psi->applyCollisionObject(collision_object);
    }

    do{
        // 等待新任务到达
        std::unique_lock<std::mutex> lock(task_mutex_);
        task_cv_.wait(lock, [this] { return has_new_task_ || cancle_current_task.load(); });
        
        if (cancle_current_task.load()) {
            RCLCPP_INFO(node->get_logger(), "任务被取消，重置状态");
            has_new_task_ = false;
            current_task_type = 0;
            continue;
        }
        
        has_new_task_ = false;
        lock.unlock();
        
        move_group_interface->setStartStateToCurrentState();
        
        // 根据任务类型执行适当的任务
        bool task_result = false;
        switch(static_cast<int>(current_task_type.load())) {
            case ArmTask::ROBOTIC_ARM_TASK_MOVE:
                task_result = execute_move_task();
                break;
            case ArmTask::ROBOTIC_ARM_TASK_CATCH_TARGET:
                task_result = execute_catch_task();
                break;
            case ArmTask::ROBOTIC_ARM_TASK_PLACE_TARGET:
                task_result = execute_place_task();
                break;
            default:
                RCLCPP_ERROR(node->get_logger(), "未知的任务类型: %d", current_task_type.load());
                task_result = false;
                break;
        }
        
        // 通过动作服务器发送最终结果
        if(current_goal_handle) {
            auto finish_msg = std::make_shared<robot_interfaces::action::Catch::Result>();
            finish_msg->success = task_result;
            finish_msg->reason = task_result ? "Task completed" : "Task failed";
            finish_msg->kfs_num = current_kfs_num.load();
            
            if(task_result) {
                current_goal_handle->succeed(finish_msg);
                RCLCPP_INFO(node->get_logger(), "任务成功完成");
            } else {
                current_goal_handle->abort(finish_msg);
                RCLCPP_ERROR(node->get_logger(), "任务执行失败");
            }
        }
        
        // 为下一个任务重置任务状态
        reset_task_state();
        
    } while(false); // 目前每个请求只执行一个任务



}


/**
 * @brief 计算接近和抓取的目标位姿。
 * 
 * 此函数基于以下因素计算最佳的接近和抓取位姿：
 * - 物体位置和方向
 * - 表面法线计算
 * - 机器人运动学约束
 * - 碰撞避免考虑
 * 
 * 算法：
 * 1. 提取物体中心和方向
 * 2. 从物体方向计算表面法线
 * 3. 根据机器人位置确定接近方向
 * 4. 计算带有适当偏移的抓取和准备位置
 * 5. 计算最佳抓取的末端执行器方向
 * 
 * @param box_pos 目标物体的位置和方向
 * @param approach_distance 接近位姿距离抓取位置的距离
 * @param grasp_pose 计算的抓取位姿的输出参数
 * @param mode 接近模式（AUTO或POS）
 * @return 初始定位的计算接近位姿
 */
geometry_msgs::msg::Pose RoboticTask::calculate_target_pose(
    const geometry_msgs::msg::Pose& box_pos, 
    double approach_distance, 
    geometry_msgs::msg::Pose& grasp_pose, 
    int mode 
){
    RCLCPP_INFO(
        node->get_logger(),"传入的box_pos: Pos(%lf, %lf, %lf), Ori(%lf, %lf, %lf, %lf)", 
        box_pos.position.x, box_pos.position.y, box_pos.position.z, 
        box_pos.orientation.x, box_pos.orientation.y, box_pos.orientation.z, box_pos.orientation.w
    );
    
    // 从输入位姿提取物体中心和方向
    Eigen::Vector3d object_center(box_pos.position.x, box_pos.position.y, box_pos.position.z);
    Eigen::Quaterniond object_quat(box_pos.orientation.w, box_pos.orientation.x, box_pos.orientation.y, box_pos.orientation.z);
    Eigen::Matrix3d object_rot = object_quat.toRotationMatrix(); // 转换为旋转矩阵

    // 计算表面法线（假设Z轴是物体的向上方向）
    Eigen::Vector3d surface_normal = object_rot * Eigen::Vector3d(0.0, 0.0, 1.0);
    RCLCPP_INFO(node->get_logger(), "表面法线 = (%f, %f, %f)", surface_normal.x(), surface_normal.y(), surface_normal.z());
    
    // 计算从机器人基座到物体的方向
    Eigen::Vector3d to_robot_base = -object_center;
    
    // 确保表面法线指向机器人
    if(surface_normal.dot(to_robot_base) < 0.0){  // dot 计算点积
        surface_normal = -surface_normal;
        RCLCPP_INFO(node->get_logger(), "反转表面法线为 (%f, %f, %f)", surface_normal.x(), surface_normal.y(), surface_normal.z());
    }

    // 计算表面位置和抓取/准备位置
    const double object_half_size = 0.175;
    Eigen::Vector3d surface_position = object_center + object_half_size * surface_normal;
    RCLCPP_INFO(node->get_logger(), "表面位置 = (%f, %f, %f)", surface_position.x(), surface_position.y(), surface_position.z());

    const double inside_offset = 0.02;  // 抓取的内部偏移
    const double outside_offset = 0.05;  // 接近的外部偏移

    Eigen::Vector3d grasp_position = surface_position - inside_offset * surface_normal;
    Eigen::Vector3d prepare_position = surface_position + outside_offset * surface_normal;

    RCLCPP_INFO(node->get_logger(), "抓取位置 = (%f, %f, %f)", grasp_position.x(), grasp_position.y(), grasp_position.z());
    RCLCPP_INFO(node->get_logger(), "准备位置 = (%f, %f, %f)", prepare_position.x(), prepare_position.y(), prepare_position.z());

    // 计算最佳接近的机器人到物体方向向量
    Eigen::Vector3d robot_base(0.0, 0.0, 0.0);
    Eigen::Vector3d robot_to_object = object_center - robot_base;
    Eigen::Vector3d robot_side_direction;
    robot_side_direction = robot_to_object;
    robot_side_direction.z() = 0;  // 投影到XY平面
    robot_side_direction.normalize();

    Eigen::Vector3d away_from_robot = -robot_side_direction;

    RCLCPP_INFO(
        node->get_logger(), 
        "机器人方向向量（从机器人看物体） = (%f, %f, %f)", robot_to_object.x(), robot_to_object.y(), robot_to_object.z()
    );
    
    RCLCPP_INFO(
        node->get_logger(), 
        "机器人侧方向向量（垂直于机器人，从机器人到物体） = (%f, %f, %f)", robot_side_direction.x(), robot_side_direction.y(), robot_side_direction.z()
    );

    RCLCPP_INFO(
        node->get_logger(), 
        "机器人远离方向向量 = (%f, %f, %f)", away_from_robot.x(), away_from_robot.y(), away_from_robot.z()
    );

    // 计算最佳吸杯对齐的末端执行器方向
    Eigen::Vector3d eef_x_axis;
    Eigen::Vector3d eef_y_axis;
    Eigen::Vector3d eef_z_axis;

    // 计算吸杯方向（垂直于表面法线，指向远离机器人的方向）
    Eigen::Vector3d suction_dir = away_from_robot - away_from_robot.dot(surface_normal) * surface_normal;
    suction_dir.normalize();

    RCLCPP_INFO(
        node->get_logger(), 
        "吸盘方向计算 = (%f, %f, %f)", suction_dir.x(), suction_dir.y(), suction_dir.z()
    );

    // 验证吸杯方向对齐
    double normal_alignment = std::abs(suction_dir.dot(surface_normal));
    RCLCPP_INFO(node->get_logger(), "吸盘法线对齐度（应为0） = %f", normal_alignment);
    RCLCPP_INFO(node->get_logger(), "吸盘方向z分量: %f （应接近零）", suction_dir.z());

    double away_alignment = suction_dir.dot(away_from_robot);
    RCLCPP_INFO(node->get_logger(), "吸盘远离方向对齐度（应为0） = %f", away_alignment);

    // 如果主要计算失败，使用备用计算
    if(normal_alignment > 0.1 || away_alignment < 0.9 || std::abs(suction_dir.z()) > 0.1){
        RCLCPP_ERROR(node->get_logger(), "吸盘方向计算错误");
        RCLCPP_WARN(node->get_logger(), "使用备用方案计算吸盘方向");

        // 备用方案：使用与全局Z轴的叉积
        Eigen::Vector3d temp = surface_normal.cross(Eigen::Vector3d(0.0, 0.0, 1.0));
        if(temp.norm() < 1e-6){
            temp = surface_normal.cross(Eigen::Vector3d(1.0, 0.0, 0.0));
        }

        temp.normalize();

        if(temp.dot(away_from_robot) < 0.0){
            suction_dir = -temp;
        } else {
            suction_dir = temp;
        }
    }

    // 设置末端执行器X轴为吸杯方向
    eef_x_axis = suction_dir;
    RCLCPP_INFO(node->get_logger(), "最终吸盘方向 = (%f, %f, %f)", eef_x_axis.x(), eef_x_axis.y(), eef_x_axis.z());

    // 使用与全局轴的叉积计算末端执行器Y轴
    Eigen::Vector3d global_x(1.0, 0.0, 0.0);
    Eigen::Vector3d global_y(0.0, 1.0, 0.0);
    Eigen::Vector3d global_z(0.0, 0.0, 1.0);

    if(std::abs(eef_x_axis.dot(global_x)) > 0.9){
        eef_y_axis = global_y.cross(eef_x_axis);
    } else {
        eef_y_axis = global_x.cross(eef_x_axis);
    }

    // 如果叉积结果为零向量，则使用备用方案
    if(eef_y_axis.norm() < 1e-6){
        eef_y_axis = global_z.cross(eef_x_axis);
    }
    eef_y_axis.normalize();

    RCLCPP_INFO(node->get_logger(), "末端Y轴方向 = (%f, %f, %f)", eef_y_axis.x(), eef_y_axis.y(), eef_y_axis.z());
    
    // 计算Z轴作为X和Y轴的叉积
    eef_z_axis = eef_x_axis.cross(eef_y_axis);
    RCLCPP_INFO(node->get_logger(), "末端Z轴方向 = (%f, %f, %f)", eef_z_axis.x(), eef_z_axis.y(), eef_z_axis.z());
    
    // 从计算的轴构建旋转矩阵
    Eigen::Matrix3d R_eef;
    R_eef.col(0) = eef_x_axis;
    R_eef.col(1) = eef_y_axis;
    R_eef.col(2) = eef_z_axis;
    Eigen::Quaterniond q_eef(R_eef);

    // 构建接近位置的结果位姿
    geometry_msgs::msg::Pose result;
    result.position.x = prepare_position.x();
    result.position.y = prepare_position.y();
    result.position.z = prepare_position.z();
    result.orientation.x = q_eef.x();
    result.orientation.y = q_eef.y();
    result.orientation.z = q_eef.z();
    result.orientation.w = q_eef.w();

    // 设置输出抓取位姿参数
    grasp_pose.position.x = grasp_position.x();
    grasp_pose.position.y = grasp_position.y();
    grasp_pose.position.z = grasp_position.z(); 
    grasp_pose.orientation.x = q_eef.x();
    grasp_pose.orientation.y = q_eef.y();
    grasp_pose.orientation.z = q_eef.z();
    grasp_pose.orientation.w = q_eef.w();
    
    // 记录计算结果用于调试
    RCLCPP_INFO(
        node->get_logger(), 
        "======================= 计算结果汇总 ========================"
    );
    RCLCPP_INFO(
        node->get_logger(), 
        "抓取位姿 = Pos(%f, %f, %f), Ori(%f, %f, %f, %f)", 
        grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z,
        grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w
    );
    RCLCPP_INFO(
        node->get_logger(), 
        "准备位姿 = Pos(%f, %f, %f), Ori(%f, %f, %f, %f)", 
        result.position.x, result.position.y, result.position.z,
        result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w
    );

    // 验证接近向量
    Eigen::Vector3d diff = prepare_position - grasp_position;
    RCLCPP_INFO(node->get_logger(), "准备位置-抓取位置向量 = (%f, %f, %f)",
        diff.x(), diff.y(), diff.z());

    double robot_alignment = eef_x_axis.dot(away_from_robot);
    RCLCPP_INFO(node->get_logger(),"吸盘于远离机器人方向对齐度 = %f(应为1)", robot_alignment);

    return result;
}


/**
 * @brief 计算准备位姿（使用固定方向）。
 * 
 * 简化的位姿计算函数，使用固定方向沿X轴进行接近。该函数适用于测试或物体方向不重要的情况。
 * 
 * @param box_pos 目标物体的位置和方向
 * @param approach_distance 接近位姿距离抓取位置的距离
 * @param grasp_pose 计算的抓取位姿的输出参数
 * @param mode 接近模式（当前未使用）
 * @return 计算的准备位姿
 */
geometry_msgs::msg::Pose RoboticTask::calculate_prepare_pose_with_orientation(
    const geometry_msgs::msg::Pose& box_pos, 
    double approach_distance, 
    geometry_msgs::msg::Pose &grasp_pose, 
    int mode)
{
    // 提取物体中心和方向
    Eigen::Vector3d object_center(box_pos.position.x, box_pos.position.y, box_pos.position.z);
    Eigen::Quaterniond q(box_pos.orientation.w, box_pos.orientation.x, 
                         box_pos.orientation.y, box_pos.orientation.z);
    
    // 使用固定方向沿X轴进行接近
    Eigen::Vector3d approach_direction(1.0, 0.0, 0.0);
    
    // 计算抓取和准备位置（使用固定偏移）
    const double object_half_size_x = 0.35;
    Eigen::Vector3d grasp_position = object_center - object_half_size_x * approach_direction;
    Eigen::Vector3d prepare_position = grasp_position + approach_distance * approach_direction;
    
    // 使用单位四元数表示末端执行器方向
    Eigen::Quaterniond q_eef(1.0, 0.0, 0.0, 0.0);
    
    // 构造结果位姿（准备位置）
    geometry_msgs::msg::Pose result;
    result.position.x = prepare_position.x();
    result.position.y = prepare_position.y();
    result.position.z = prepare_position.z();
    result.orientation.w = q_eef.w();
    result.orientation.x = q_eef.x();
    result.orientation.y = q_eef.y();
    result.orientation.z = q_eef.z();
    
    // 设置输出抓取位姿参数
    grasp_pose.position.x = grasp_position.x();
    grasp_pose.position.y = grasp_position.y();
    grasp_pose.position.z = grasp_position.z();
    grasp_pose.orientation.w = q_eef.w();
    grasp_pose.orientation.x = q_eef.x();
    grasp_pose.orientation.y = q_eef.y();
    grasp_pose.orientation.z = q_eef.z();
    
    return result;
}


/**
 * @brief 向机器人添加附加的KFS碰撞对象。
 * 
 * 创建并附加一个代表KFS（运动反馈系统）的碰撞对象到机器人的link6。
 * 这用于模拟成功抓取后机器人携带的物体。
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::add_attached_kfs_collision(){
    moveit_msgs::msg::AttachedCollisionObject collision_object;
    collision_object.link_name = "link6";
    collision_object.object.header.frame_id = "link6";
    collision_object.object.id = "kfs";
    
    // 为KFS对象定义盒子形状
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.35;
    primitive.dimensions[primitive.BOX_Y] = 0.35;
    primitive.dimensions[primitive.BOX_Z] = 0.35;

    collision_object.object.primitives.push_back(primitive);
    collision_object.object.primitive_poses.push_back(attached_kfs_pos);
    collision_object.object.operation = collision_object.object.ADD;
    
    // 将附加碰撞对象应用到规划场景
    psi->applyAttachedCollisionObject(collision_object);
    return true;
}


/**
 * @brief 从机器人移除附加的KFS碰撞对象。
 * 
 * 从link6移除附加的KFS碰撞对象，同时也从规划场景中移除任何对应的碰撞对象。
 * 这通常在释放物体后调用。
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::remove_attached_kfs_collision() {
    moveit_msgs::msg::AttachedCollisionObject collision_object;
    collision_object.link_name = "link6";
    collision_object.object.header.frame_id = "link6";
    collision_object.object.id = "kfs";
    collision_object.object.operation = collision_object.object.REMOVE;
    
    // Remove attached collision object
    psi->applyAttachedCollisionObject(collision_object);

    // 同时移除具有相同ID的独立碰撞对象
    remove_kfs_collision("kfs", move_group_interface->getPlanningFrame());
    return true;
}


/**
 * @brief 向规划场景添加KFS碰撞对象。
 * 
 * 添加一个代表KFS的独立碰撞对象到规划场景。
 * 这用于表示环境中机器人需要避免或与之交互的物体。
 * 
 * @param pose 碰撞对象的位置和方向
 * @param object_id 对象的唯一标识符
 * @param frame_id 对象位置的参考坐标系
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::add_kfs_collision(
    const geometry_msgs::msg::Pose& pose, 
    const std::string& object_id, 
    const std::string& frame_id 
){
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = object_id;
    
    // 为碰撞对象定义盒子形状
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3); 
    primitive.dimensions[primitive.BOX_X] = 0.35;
    primitive.dimensions[primitive.BOX_Y] = 0.35;
    primitive.dimensions[primitive.BOX_Z] = 0.35;
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    
    // 将碰撞对象添加到规划场景
    psi->applyCollisionObject(collision_object);
    return true;
}


/**
 * @brief 从规划场景移除KFS碰撞对象。
 * 
 * 使用唯一标识符从规划场景中移除独立碰撞对象。
 * 
 * @param object_id 要移除对象的唯一标识符
 * @param frame_id 对象的参考坐标系
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::remove_kfs_collision(
    const std::string& object_id, 
    const std::string& frame_id
){
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = object_id;
    collision_object.operation = collision_object.REMOVE;
    
    // 从规划场景移除碰撞对象
    psi->applyCollisionObject(collision_object);
    return true;
}


/**
 * @brief 发送关节速度命令到硬件驱动节点
 * 
 * 将计算出的关节速度通过ROS2话题发布给硬件驱动节点，
 * 实现对机械臂的速度控制。
 * 
 * @param joint_velocities 要发送的关节速度向量
 * @return 发送成功返回true，失败返回false
 */
bool RoboticTask::send_joint_velocity_to_hardware(const Eigen::VectorXd& joint_velocities) {
    if (joint_velocities.size() != 6) {
        RCLCPP_ERROR(node->get_logger(), "关节速度向量维度错误，期望6维，实际%zu维", joint_velocities.size());
        return false;
    }

    // 创建ROS2消息
    auto velocity_msg = std::make_shared<robot_interfaces::msg::Robot>();
    
    // 填充关节速度数据
    // joints 是 std::array，大小固定为6，无需 resize
    
    for (int i = 0; i < 6; ++i) {
        velocity_msg->joints[i].rad = joint_position[i];    // 当前位置
        velocity_msg->joints[i].omega = joint_velocities[i]; // 目标速度
        velocity_msg->joints[i].torque = 0.0f;              // 力矩设为0（纯速度控制）
        velocity_msg->joints[i].alpha = 0.0f;               // 加速度设为0
    }
    
        
    // 发布速度命令
    joint_velocity_publisher_->publish(*velocity_msg);
    
    RCLCPP_DEBUG(node->get_logger(), "发送关节速度命令: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 joint_velocities[0], joint_velocities[1], joint_velocities[2],
                 joint_velocities[3], joint_velocities[4], joint_velocities[5]);
    
    return true;
}

/**
 * @brief 发送关节力矩命令到硬件驱动节点
 * 
 * 将计算出的关节力矩通过ROS2话题发布给硬件驱动节点，
 * 实现对机械臂的力矩控制。
 * 
 * @param joint_torques 要发送的关节力矩向量
 * @return 发送成功返回true，失败返回false
 */
bool RoboticTask::send_joint_torque_to_hardware(const Eigen::VectorXd& joint_torques) {
    if (joint_torques.size() != 6) {
        RCLCPP_ERROR(node->get_logger(), "关节力矩向量维度错误，期望6维，实际%zu维", joint_torques.size());
        return false;
    }

    // 创建ROS2消息
    auto torque_msg = std::make_shared<robot_interfaces::msg::Robot>();
    
    // 填充关节力矩数据
    for (int i = 0; i < 6; ++i) {
        torque_msg->joints[i].rad = joint_position[i];         // 当前位置
        torque_msg->joints[i].omega = 0.0f;                     // 速度设为0（纯力矩控制）
        torque_msg->joints[i].torque = static_cast<float>(joint_torques[i]); // 目标力矩
        torque_msg->joints[i].alpha = 0.0f;                     // 加速度设为0
    }
    
    // 发布力矩命令
    joint_velocity_publisher_->publish(*torque_msg);
    
    RCLCPP_DEBUG(node->get_logger(), "发送关节力矩命令: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 joint_torques[0], joint_torques[1], joint_torques[2],
                 joint_torques[3], joint_torques[4], joint_torques[5]);
    
    return true;
}


/**
 * @brief 设置气泵启用/禁用状态。
 * 
 * 通过在驱动节点上设置'enable_air_pump'参数来控制气泵（吸盘）。
 * 包含重试机制以提高可靠性。
 * 
 * @param enable true启用气泵，false禁用气泵
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::set_air_pump(bool enable){
    RCLCPP_INFO(node->get_logger(), "设置气泵参数: %s", enable ? "开启" : "关闭");

    // 检查驱动节点是否可用
    auto node_names = node->get_node_names();
    if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
        RCLCPP_ERROR(node->get_logger(), "未找到气泵驱动节点");
        return false;
    }

    // 为驱动节点创建参数客户端
    auto temp_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/driver_node");

    // 等待服务可用
    if(!temp_client->wait_for_service(1s)){
        RCLCPP_ERROR(node->get_logger(), "驱动节点参数服务不可用");
        return false;
    }

    // 添加重试机制以提高可靠性
    const int max_retries = 3;
    for (int retry = 0; retry < max_retries; ++retry) {
        auto future = temp_client->set_parameters({
            rclcpp::Parameter("enable_air_pump", enable)
        });

        try{
            const auto& result = future.get();

            if (result.empty()){
                RCLCPP_ERROR(node->get_logger(), "返回结果为空，重试 %d/%d", retry + 1, max_retries);
                continue;
            }

            const auto& first_result = result.front();
            if(first_result.successful){
                RCLCPP_INFO(node->get_logger(), "气泵设置成功");
                return true;
            } else {
                RCLCPP_ERROR(node->get_logger(), "设置失败: %s，重试 %d/%d", 
                             first_result.reason.c_str(), retry + 1, max_retries);
            }
        } catch (const std::exception& e){
            RCLCPP_ERROR(node->get_logger(), "设置失败: %s，重试 %d/%d", e.what(), retry + 1, max_retries);
        }
        
        // 重试前等待（最后一次尝试除外）
        if (retry < max_retries - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    RCLCPP_ERROR(node->get_logger(), "气泵设置最终失败");
    return false;
}


/**
 * @brief 验证气泵状态是否匹配期望值。
 * 
 * 通过从驱动节点读取'enable_air_pump'参数来检查实际的气泵状态。
 * 这有助于确保气泵正确响应了命令。
 * 
 * @param expected_status 期望的气泵状态
 * @return 如果状态匹配返回true，否则返回false
 */
bool RoboticTask::verify_air_pump_status(bool expected_status) {
    // 创建参数客户端检查气泵状态
    auto temp_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/driver_node");
    
    if(!temp_client->wait_for_service(1s)) {
        RCLCPP_WARN(node->get_logger(), "无法连接到驱动节点验证气泵状态");
        return true; // 假设成功以避免阻塞
    }
    
    auto future = temp_client->get_parameters({"enable_air_pump"});
    
    try {
        const auto& result = future.get();
        if (!result.empty()) {
            bool current_status = result[0].as_bool();
            return current_status == expected_status;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node->get_logger(), "获取气泵状态异常: %s", e.what());
    }
    
    return true; // 默认返回成功以避免阻塞
}


/**
 * @brief 验证抓取操作是否成功。
 * 
 * 通过检查各种指标来验证抓取操作的成功：
 * - 关节电流变化（表示接触力）
 * - 真空传感器状态（如果可用）
 * - 末端执行器负载变化
 * 
 * 当前实现基于延迟的简单验证，但可以用更复杂的传感器扩展。
 * 
 * @return 如果抓取成功返回true，否则返回false
 */
bool RoboticTask::verify_grasp_success() {
    // 可以通过多种方式验证抓取：
    // 1. 检查关节电流变化
    // 2. 检查真空传感器（如果有）
    // 3. 检查末端执行器负载
    
    // 简单实现：等待一段时间后检查系统状态
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // TODO: 添加更复杂的验证逻辑
    // 例如：检查关节负载、真空度等
    RCLCPP_INFO(node->get_logger(), "抓取验证完成");
    return true;
}


/**
 * @brief 获取当前关节位置。
 * 
 * 返回当前关节位置向量，该向量由关节状态订阅器持续更新。
 * 
 * @return 当前关节位置向量
 */
Eigen::VectorXd RoboticTask::get_joint_position() const {
    return joint_position;
}


/**
 * @brief 关节状态更新的回调函数。
 * 
 * 当接收到新的关节状态数据时调用。使用最新的关节角度更新
 * joint_position向量，用于运动规划和控制。
 * 
 * @param msg 关节状态消息的共享指针
 */
void RoboticTask::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 从消息更新关节位置
    if (msg->position.size() >= 6) {
        for (size_t i = 0; i < 6 && i < msg->position.size(); ++i) {
            joint_position[i] = msg->position[i];
        }
    }
}


// ==================== 状态机相关函数实现 ==============
// 
//  */

/**
 * @brief 执行移动任务。
 * 
 * 通过转换以下状态来执行简单的移动任务：
 * 1. 移动到预备抓取点
 * 2. 移动到抓取点
 * 3. 返回空闲状态
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::execute_move_task() {
    RCLCPP_INFO(node->get_logger(), "开始执行移动任务");
    
    // 移动任务：直接移动到目标位置
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_READY_CATCH_POINT);
    if(!handle_move_to_ready_catch_point()) return false;
    
    // 移动任务完成，不需要抓取和释放
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_IDLE);
    return handle_idle_state();
}


/**
 * @brief 执行抓取任务。
 * 
 * 通过转换所有必要的状态来执行完整的抓取序列：
 * 1. 移动到预备抓取点
 * 2. 移动到抓取点
 * 3. 抓取目标（激活气泵）
 * 4. 移动到释放点
 * 5. 释放目标（停用气泵）
 * 6. 移动到空闲点
 * 7. 返回空闲状态
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::execute_catch_task() {
    RCLCPP_INFO(node->get_logger(), "开始执行抓取任务");
    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_READY_CATCH_POINT);
    if(!handle_move_to_ready_catch_point()) return false;
    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_CATCH_POINT);
    if(!handle_move_to_catch_point()) return false;
    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_CATCH_TARGET);
    if(!handle_catch_target()) return false;
    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_RELEASE_POINT);
    if(!handle_move_to_release_point()) return false;
    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_RELEASE_TARGET);
    if(!handle_release_target()) return false;
    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_IDLE_POINT);
    if(!handle_move_to_idle_point()) return false;

    return true;
}


/**
 * @brief 执行放置任务。
 * 
 * 执行与抓取相同的状态序列的放置任务，
 * 但释放点计算逻辑不同。
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::execute_place_task() {
    RCLCPP_INFO(node->get_logger(), "开始执行放置任务");
    
    // 与抓取任务类似的状态序列，但逻辑不同    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_CATCH_POINT);
    if(!handle_move_to_catch_point_kfs_not_zero()) return false;
    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_CATCH_TARGET);
    if(!handle_catch_target()) return false;
    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_RELEASE_POINT);
    if(!handle_move_to_release_point()) return false;
    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_RELEASE_TARGET);
    if(!handle_release_target()) return false;
    
    transition_to_state(ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_IDLE_POINT);
    if(!handle_move_to_idle_point()) return false;
    
    return true;
}


// 状态处理函数

/**
 * @brief 处理空闲状态。
 * 
 * 管理机器人等待新任务的空闲状态。
 * 更新反馈并执行最小处理。
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::handle_idle_state() {
    // 记录日志：开始处理空闲状态
    RCLCPP_INFO(node->get_logger(), "处理空闲状态");
    // 更新反馈给客户端，告知当前状态
    update_feedback();

    //**
    // 检索一：设置开始前的空闲位置。
    //*/
    Eigen::VectorXd idle_joints(6);
    idle_joints << 0.0, 0.8726646259971648, 2.1816615649929116, 2.2514747350725445, 0.0, 0.0;
    // 将Eigen向量转换为std::vector<double>，因为MoveIt接口需要此类型
    std::vector<double> idle_joints_vec(idle_joints.data(), idle_joints.data() + idle_joints.size());
    // 设置MoveIt的目标关节值
    move_group_interface->setJointValueTarget(idle_joints_vec);

    const double dt = 1.0 / dynamics_params_.control_frequency;
    
    Eigen::VectorXd current_joint_velocities = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ee_acceleration = Eigen::VectorXd::Zero(6);


    // 规划到空闲位置：使用MoveIt规划从当前位置到空闲位置的轨迹
    // 创建规划对象
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // 执行规划，获取错误码
    moveit::planning_interface::MoveItErrorCode error_code = move_group_interface->plan(plan);
    // 初始化重试计数器
    count = 0;

    // 重试循环：如果规划失败，重试最多100次
    // 这可以处理临时规划失败，如环境变化
    do {
        // 重新执行规划
        error_code = move_group_interface->plan(plan);
        // 计数器递增
        count++;
    } while (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS && count < 100);

    // 检查规划结果：如果失败，记录错误并返回false
    if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "规划到空闲位置失败");
        return false;
    }

    /**
     * @brief 使用五次多项式轨迹管理器进行平滑执行
     *
     * 五次多项式轨迹可以保证位置、速度、加速度的连续性，实现平滑运动
     */
    // 设置轨迹：将规划得到的关节轨迹传递给多项式轨迹管理器
    robot_pose_polynomial_ContinuousTrajectory.set_trajectory(plan.trajectory_.joint_trajectory);
    // 开始跟踪轨迹：记录开始时间
    robot_pose_polynomial_ContinuousTrajectory.start_track(node->now());
    // 控制循环：使用平滑轨迹点发送速度命令
    // 创建100Hz的循环率（每秒100次）
    rclcpp::Rate rate(100); // 100Hz控制频率
    // 记录轨迹开始时间
    auto start_time = node->now();

    // 计算轨迹总持续时间：从轨迹最后一个点的时间戳获取
    // 时间戳包含秒和纳秒，需要转换
    double trajectory_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +    // s
                                 plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;  // ns

    // 主控制循环：执行轨迹直到结束
    // 循环条件：当前时间距离开始时间小于轨迹持续时间加1秒缓冲
    while ((node->now() - start_time).seconds() < trajectory_duration + 1.0){

        Eigen::VectorXd current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(move_group_interface->getCurrentJointValues().data(), move_group_interface->getCurrentJointValues().size());
        Eigen::MatrixXd jacobian = velocity_ik_generator_RobotArmKinematics.computeJacobian(current_joint_positions_);


        // 获取当前时间的期望轨迹点（位置、速度、加速度）
        trajectory_msgs::msg::JointTrajectoryPoint target_point;
        // 如果成功获取目标点，则执行速度控制
        if(robot_pose_polynomial_ContinuousTrajectory.get_target(node->now(), target_point)){
            // 获取当前关节位置：从MoveIt获取实际关节反馈
            // TODO: 从实际反馈获取当前关节位置
            std::vector<double> current_joint_positions = move_group_interface->getCurrentJointValues();
            

            std::vector<double> joint_velocities(6);
            // 时间步长：10ms，对应100Hz控制频率
            double dt = 0.01;
            double kp = 100.0;
            // 对每个关节计算速度

            Eigen::VectorXd dynamics_torque = Eigen::VectorXd::Zero(6);
            bool use_torque_control = false;

            for(size_t i = 0 ; i < 6 ; ++i){
                double position_error = target_point.positions[i] - current_joint_positions[i];
                double feedforward_velocity = target_point.velocities[i];
                double feedback_velocity = kp * position_error;

                joint_velocities[i] = feedforward_velocity + feedback_velocity;
                current_joint_velocities[i] = joint_velocities[i];
            } 

            if(kdl_dynamics_ && kdl_dynamics_->isInitialized() && dynamics_params_.enable_dynamics_compensation){
                try {
                    Eigen::VectorXd gravity_compensation = kdl_dynamics_->calculateGravityCompensation(current_joint_positions_);
                    Eigen::VectorXd coriolis_compensation = kdl_dynamics_->calculateCoriolisCompensation(
                        current_joint_positions_, current_joint_velocities);
                    
                    // 从轨迹点获取关节加速度
                    if (!target_point.accelerations.empty()) {
                        Eigen::VectorXd joint_accelerations(target_point.accelerations.size());
                        for (size_t i = 0; i < target_point.accelerations.size(); ++i) {
                            joint_accelerations(i) = target_point.accelerations[i];
                        }
                        
                        // 计算完整的动力学力矩
                        dynamics_torque = kdl_dynamics_->calculateDynamicsTorque(
                            current_joint_positions_, current_joint_velocities, joint_accelerations);
                        
                        // 应用动力学补偿增益调整关节速度
                        for (size_t i = 0; i < 6; ++i) {
                            joint_velocities[i] += dynamics_params_.compensation_gain * dynamics_torque(i);
                        }
                    }
                    
                    
                }
                catch(const std::exception& e) {
                    RCLCPP_WARN(node->get_logger(), " Kinetic compensation failure %s", e.what());
                }
            }

            // 发送速度命令到控制器：将速度转换为Eigen向量
            Eigen::VectorXd joint_velocities_eigen(6);
            for(size_t i = 0 ; i < 6 ; ++i){
                joint_velocities_eigen(i) = joint_velocities[i];
            }

            // 调用硬件接口发送关节速度命令
            send_joint_velocity_to_hardware(joint_velocities_eigen);
        }
        // 睡眠以维持100Hz频率
        rate.sleep();
    }

    // 停止速度命令：轨迹执行完毕后发送零速度命令停止机器人
    Eigen::VectorXd zero_velocities(6);
    zero_velocities.setZero();  // 设置所有速度为0
    send_joint_velocity_to_hardware(zero_velocities);

    // 短暂延迟：确保停止命令生效
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // 返回成功
    return true;
}


/**
 * @brief 处理移动到预备抓取点状态。
 * 
 * 将机器人移动到预备抓取位置，该位置为实际抓取操作提供良好的可见性
 * 和接近角度。
 * 只移动到预抓取位置，后续抓取实现由视觉伺服实现
 * 
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::handle_move_to_ready_catch_point() {
    RCLCPP_INFO(node->get_logger(), "处理移动到预备抓取位置状态");
    update_feedback();
    
    // ========================================
    // 实现位置1: 移动到预备抓取位置的路径规划
    // ========================================
    // 需要实现的具体内容:
    // 1. 使用 calculate_target_pose() 计算预备抓取位置
    // 2. 使用 move_group_interface->setPoseTarget() 设置目标
    // 3. 使用 move_group_interface->plan() 规划路径
    // 4. 使用 move_group_interface->execute() 执行移动
    // 5. 检查规划结果和执行状态
    // 6. 处理可能的规划失败和重试逻辑
    
    // 当前占位符实现:
    geometry_msgs::msg::Pose grasp_pose;
    geometry_msgs::msg::Pose prepare_pose = calculate_target_pose(
        task_target_pos, 0.1, grasp_pose, static_cast<int>(ApproachMode::AUTO)
    );
    RCLCPP_INFO(node->get_logger(), "准备抓取目标位姿: Pos(%.3f, %.3f, %.3f), Ori(%.3f, %.3f, %.3f, %.3f)",
                prepare_pose.position.x, prepare_pose.position.y, prepare_pose.position.z,
                prepare_pose.orientation.x, prepare_pose.orientation.y,
                prepare_pose.orientation.z, prepare_pose.orientation.w);
    
    // 打印当前机器人状态
    auto current_joint_values = move_group_interface->getCurrentJointValues();
    auto current_pose = move_group_interface->getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "当前关节位置: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                current_joint_values[0], current_joint_values[1], current_joint_values[2],
                current_joint_values[3], current_joint_values[4], current_joint_values[5]);
    RCLCPP_INFO(node->get_logger(), "当前末端位姿: Pos(%.3f, %.3f, %.3f), Ori(%.3f, %.3f, %.3f, %.3f)",
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                current_pose.pose.orientation.x, current_pose.pose.orientation.y, 
                current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    
    move_group_interface->setStartStateToCurrentState();
    move_group_interface->setPoseTarget(prepare_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    count = 0 ;
    auto success = move_group_interface->plan(plan);
    RCLCPP_INFO(node->get_logger(), "首次规划结果: %s", 
                success == moveit::core::MoveItErrorCode::SUCCESS ? "成功" : "失败");
    
    do{
        success = move_group_interface->plan(plan);
        count ++;
        RCLCPP_INFO(node->get_logger(), "规划尝试 %d: %s", count,
                    success == moveit::core::MoveItErrorCode::SUCCESS ? "成功" : "失败");
    } while (success != moveit::core::MoveItErrorCode::SUCCESS && count < MAX_COUNT);
    if (success != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "预备抓取位置规划失败，放弃执行");
        move_group_interface->clearPoseTargets();
        return false;
    }
    if (plan.trajectory_.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(node->get_logger(), "规划轨迹为空，放弃执行");
        move_group_interface->clearPoseTargets();
        return false;
    }
    move_group_interface->clearPoseTargets();
    
    const double dt = 1.0 / dynamics_params_.control_frequency;
    Eigen::VectorXd current_joint_velocities = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ee_acceleration = Eigen::VectorXd::Zero(6);
    robot_pose_polynomial_ContinuousTrajectory.set_trajectory(plan.trajectory_.joint_trajectory);
    robot_pose_polynomial_ContinuousTrajectory.start_track(node->now());
    rclcpp::Rate rate(100);
    auto start_time = node->now();
    double trajectory_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + 
                                 plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
    while ((node->now() - start_time).seconds() < trajectory_duration + 1.0){
        Eigen::VectorXd current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(
            move_group_interface->getCurrentJointValues().data(), move_group_interface->getCurrentJointValues().size());
        Eigen::MatrixXd jacobian = velocity_ik_generator_RobotArmKinematics.computeJacobian(current_joint_positions_);
        trajectory_msgs::msg::JointTrajectoryPoint target_point;
        if(robot_pose_polynomial_ContinuousTrajectory.get_target(node->now(), target_point)){
            std::vector<double> current_joint_positions = move_group_interface->getCurrentJointValues();
            std::vector<double> joint_velocities(6);
            double dt = 0.01;
            double kp = 100.0;
            Eigen::VectorXd dynamics_torque = Eigen::VectorXd::Zero(6);
            bool use_torque_control = false;
            for(size_t i = 0 ; i < 6 ; ++i){
                double position_error = target_point.positions[i] - current_joint_positions[i];
                double feedforward_velocity = target_point.velocities[i];
                double feedback_velocity = kp * position_error;
                joint_velocities[i] = feedforward_velocity + feedback_velocity;
                current_joint_velocities[i] = joint_velocities[i];
            }

            if(kdl_dynamics_ && kdl_dynamics_->isInitialized() && dynamics_params_.enable_dynamics_compensation){
                try{
                    Eigen::VectorXd gravity_compensation = kdl_dynamics_->calculateGravityCompensation(current_joint_positions_);
                    Eigen::VectorXd coriolis_compensation = kdl_dynamics_->calculateCoriolisCompensation(
                        current_joint_positions_, current_joint_velocities);
                    if (!target_point.accelerations.empty()){
                        Eigen::VectorXd joint_acceleration(target_point.accelerations.size());
                        for(size_t i = 0 ; i < target_point.accelerations.size(); ++i){
                            joint_acceleration[i] = target_point.accelerations[i];
                        }
                        dynamics_torque = kdl_dynamics_->calculateDynamicsTorque(
                            current_joint_positions_, current_joint_velocities, joint_acceleration);

                        for(size_t i = 0 ; i < 6 ; ++i){
                            joint_velocities[i] += dynamics_params_.compensation_gain * dynamics_torque(i);
                        }
                    }
                } catch (const std::exception& e){
                    RCLCPP_WARN(node->get_logger(), "Kinetic compensation failure %s", e.what());
                }
            }

            Eigen::VectorXd joint_velocities_eigen(6);
            for(size_t i = 0 ; i < 6 ; ++i){
                joint_velocities_eigen(i) = joint_velocities[i];
            }

            send_joint_velocity_to_hardware(joint_velocities_eigen);

        }

        rate.sleep();
    }    

    Eigen::VectorXd zero_velocities(6);
    zero_velocities.setZero();
    send_joint_velocity_to_hardware(zero_velocities);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    return !cancle_current_task.load();
}


/**
 * @brief 处理移动到抓取点状态。
 * 
 * 将机器人从预备位置移动到实际抓取位置。
 * 这需要更精确的定位，使用视觉伺服
 * 进行最终接近。
 * 
 * TODO: 使用视觉伺服实现精细路径规划
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::handle_move_to_catch_point() {
    RCLCPP_INFO(node->get_logger(), "处理移动到抓取位置状态");
    update_feedback();
    
    // ========================================
    // 实现位置2: 移动到抓取位置的路径规划
    // ========================================
    // 需要实现的具体内容:
    // 1. 使用 calculate_target_pose() 计算精确抓取位置
    // 2. 设置更精确的位置和方向约束
    // 3. 使用 move_group_interface->setPoseTarget() 设置目标
    // 4. 考虑避障和碰撞检测
    // 5. 实现更精细的路径规划（可能需要视觉伺服）
    // 6. 添加接近速度控制
    
    // 当前占位符实现:
    geometry_msgs::msg::Pose grasp_pose;
    geometry_msgs::msg::Pose prepare_pose = calculate_target_pose(
        task_target_pos, 0.05, grasp_pose, static_cast<int>(ApproachMode::POS)
    );
    
    // 使用计算出的抓取位置而不是预备位置
    // 基于速度控制实现视觉伺服，添加KDL动力学补偿
    /**
     * target_object_position 作为传入量
     * 调用calculate_end_effector_velocity  calculate_joint_velocity
     * 在吸取过程中启动气泵
     * 
     * 新增功能：
     * - 使用KDL动力学补偿提高控制精度
     * - 实现末端恒定加速度运动
     * - 添加关节加速度控制
     * - 安全检查和紧急停止
     */

    // 控制参数
    const double dt = 1.0 / dynamics_params_.control_frequency;
    
    // 获取当前关节位置和速度
    Eigen::VectorXd current_joint_positions = get_joint_position();
    Eigen::VectorXd current_joint_velocities = Eigen::VectorXd::Zero(6);
    
    // 计算雅可比矩阵用于加速度映射
    Eigen::MatrixXd jacobian = velocity_ik_generator_RobotArmKinematics.computeJacobian(current_joint_positions);
    
    // 末端恒定加速度向量
    Eigen::Vector3d ee_acceleration = target_object_position.normalized(); // 向下恒定加速度
    
    while (true) {
        // 计算末端速度
        calculate_end_effector_velocity(
            target_object_position.x(), 
            target_object_position.y(), 
            target_object_position.z()
        );
        
        // 基础关节速度计算
        auto joint_velocities = calculate_joint_velocity(end_effector_velocity);
        
        // 动力学力矩变量（需要在if块外定义）
        Eigen::VectorXd dynamics_torque = Eigen::VectorXd::Zero(6);
        bool use_torque_control = false;
        
        // 如果KDL动力学可用且启用动力学补偿，添加动力学补偿
        if (kdl_dynamics_ && kdl_dynamics_->isInitialized() && dynamics_params_.enable_dynamics_compensation) {
            try {
                // 计算重力补偿
                Eigen::VectorXd gravity_compensation = kdl_dynamics_->calculateGravityCompensation(current_joint_positions);
                
                // 计算科氏力补偿
                Eigen::VectorXd coriolis_compensation = kdl_dynamics_->calculateCoriolisCompensation(
                    current_joint_positions, current_joint_velocities);
                
                // 计算关节加速度（基于末端恒定加速度）
                Eigen::VectorXd joint_accelerations = kdl_dynamics_->calculateJointAcceleration(
                    current_joint_positions, current_joint_velocities, ee_acceleration, jacobian);
                
                // 限制关节加速度
                for (int i = 0; i < joint_accelerations.size(); ++i) {
                    joint_accelerations(i) = std::max(-dynamics_params_.max_joint_acceleration, 
                                                   std::min(dynamics_params_.max_joint_acceleration, joint_accelerations(i)));
                }
                
                // 计算完整的动力学力矩
                dynamics_torque = kdl_dynamics_->calculateDynamicsTorque(
                    current_joint_positions, current_joint_velocities, joint_accelerations);
                
                // 安全检查：关节力矩限制
                if (!checkJointTorqueLimits(dynamics_torque)) {
                    RCLCPP_WARN(node->get_logger(), "关节力矩超出限制，停止运动");
                    break;
                }
                
                // 安全检查：紧急停止条件
                if (dynamics_params_.enable_acceleration_control && 
                    checkEmergencyStop(current_joint_positions, joint_velocities, joint_accelerations)) {
                    RCLCPP_ERROR(node->get_logger(), "触发紧急停止条件");
                    cancle_current_task = true;
                    break;
                }
                
                // 使用完整的动力学前馈补偿
                Eigen::VectorXd tau_ff = dynamics_torque; // 包含惯性、科氏力、重力
                
                // 将动力学补偿转换为速度调整
                for (int i = 0; i < joint_velocities.size(); ++i) {
                    // 使用补偿增益将力矩转换为速度修正量
                    // 简化实现：τ ≈ K_v * Δv，其中 K_v 是速度增益
                    joint_velocities(i) += dynamics_params_.compensation_gain * tau_ff(i);
                }

                // use_torque_control = true;
                
                // 更新关节速度状态
                current_joint_velocities = joint_velocities;
                current_joint_positions += joint_velocities * dt;
                
            } catch (const std::exception& e) {
                RCLCPP_WARN(node->get_logger(), "KDL动力学计算失败: %s", e.what());
                // 降级到基础控制
                use_torque_control = false;
            }
        }
        
        // 根据控制模式发送命令
        if (use_torque_control) {
            // 发送力矩命令（使用计算出的动力学力矩）
            send_joint_torque_to_hardware(dynamics_torque);
        } else {
            // 发送速度命令（降级模式）
            send_joint_velocity_to_hardware(joint_velocities);
        }
        
        
        // 控制频率等待
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
        
        // 检查停止条件
        if (cancle_current_task.load() || end_effector_velocity.norm() < 0.01) {
            break;
        }
    }
    
    // 停止力矩命令：轨迹执行完毕后发送零力矩命令停止机器人
    Eigen::VectorXd zero_torques(6);
    zero_torques.setZero();  // 设置所有力矩为0
    send_joint_torque_to_hardware(zero_torques);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    return !cancle_current_task.load();
}



/**
 * @brief 处理抓取车上物块
 * 
 * 根据current_kfs_num设置抓取点
 *
 * TODO: 实现抓取车上物块的逻辑,实际应用中的抓取位置
 */
bool RoboticTask::handle_move_to_catch_point_kfs_not_zero(){
    switch (current_kfs_num.load()) {
        case 0:
            RCLCPP_ERROR(node->get_logger(), "KFS == 0, 没有KFS可供 抓取");
            break;

        //**
        // 检索四：抓取车上KFS位置
        //*/ 
        case 1:
            {
            Eigen::VectorXd kfs1_touch_pos(6);
            kfs1_touch_pos << -0.087266463, 0.174532925, 4.677482396, -0.122173048, -0.017453293, 0.000000000;
            std::vector<double> kfs1_touch_pos_vec(kfs1_touch_pos.data(), kfs1_touch_pos.data() + kfs1_touch_pos.size());
            move_group_interface->setJointValueTarget(kfs1_touch_pos_vec);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            count = 0;
            auto error_code = move_group_interface->plan(plan);
            do{
                error_code = move_group_interface->plan(plan);
                count ++ ;
            } while (error_code != moveit::core::MoveItErrorCode::SUCCESS && count < MAX_COUNT);

            if(error_code != moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_ERROR(node->get_logger(), "抓取车上物块路径规划失败");
                return false;
            }

            const double dt = 1.0 / dynamics_params_.control_frequency;
            Eigen::VectorXd current_joint_velocities = Eigen::VectorXd::Zero(6);
            Eigen::VectorXd ee_acceleration = Eigen::VectorXd::Zero(6);
            robot_pose_polynomial_ContinuousTrajectory.set_trajectory(plan.trajectory_.joint_trajectory);
            robot_pose_polynomial_ContinuousTrajectory.start_track(node->now());
            rclcpp::Rate rate(100);
            auto start_time = node->now();
            double trajectory_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + 
                                        plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
            while ((node->now() - start_time).seconds() < trajectory_duration + 1.0){
                Eigen::VectorXd current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(
                    move_group_interface->getCurrentJointValues().data(), move_group_interface->getCurrentJointValues().size());
                Eigen::MatrixXd jacobian = velocity_ik_generator_RobotArmKinematics.computeJacobian(current_joint_positions_);
                trajectory_msgs::msg::JointTrajectoryPoint target_point;
                if(robot_pose_polynomial_ContinuousTrajectory.get_target(node->now(), target_point)){
                    std::vector<double> current_joint_positions = move_group_interface->getCurrentJointValues();
                    std::vector<double> joint_velocities(6);
                    double dt = 0.01;
                    double kp = 100.0;
                    Eigen::VectorXd dynamics_torque = Eigen::VectorXd::Zero(6);
                    bool use_torque_control = false;
                    for(size_t i = 0 ; i < 6 ; ++i){
                        double position_error = target_point.positions[i] - current_joint_positions[i];
                        double feedforward_velocity = target_point.velocities[i];
                        double feedback_velocity = kp * position_error;
                        joint_velocities[i] = feedforward_velocity + feedback_velocity;
                        current_joint_velocities[i] = joint_velocities[i];
                    }

                    if(kdl_dynamics_ && kdl_dynamics_->isInitialized() && dynamics_params_.enable_dynamics_compensation){
                        try{
                            Eigen::VectorXd gravity_compensation = kdl_dynamics_->calculateGravityCompensation(current_joint_positions_);
                            Eigen::VectorXd coriolis_compensation = kdl_dynamics_->calculateCoriolisCompensation(
                                current_joint_positions_, current_joint_velocities);
                            if (!target_point.accelerations.empty()){
                                Eigen::VectorXd joint_acceleration(target_point.accelerations.size());
                                for(size_t i = 0 ; i < target_point.accelerations.size(); ++i){
                                    joint_acceleration[i] = target_point.accelerations[i];
                                }
                                dynamics_torque = kdl_dynamics_->calculateDynamicsTorque(
                                    current_joint_positions_, current_joint_velocities, joint_acceleration);

                                for(size_t i = 0 ; i < 6 ; ++i){
                                    joint_velocities[i] += dynamics_params_.compensation_gain * dynamics_torque(i);
                                }
                            }
                        } catch (const std::exception& e){
                            RCLCPP_WARN(node->get_logger(), "Kinetic compensation failure %s", e.what());
                        }
                    }

                    Eigen::VectorXd joint_velocities_eigen(6);
                    for(size_t i = 0 ; i < 6 ; ++i){
                        joint_velocities_eigen(i) = joint_velocities[i];
                    }

                    send_joint_velocity_to_hardware(joint_velocities_eigen);

                }

                rate.sleep();
            }    

            Eigen::VectorXd zero_velocities(6);
            zero_velocities.setZero();
            send_joint_velocity_to_hardware(zero_velocities);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            return true;

            break;
            }
        case 2:
            {
            Eigen::VectorXd kfs2_touch_pos(6);
            kfs2_touch_pos << -0.087266463, 0.296705972, 3.769911185, 0.645771823, -0.087266463, 0.000000000;
            std::vector<double> kfs2_touch_pos_vec(kfs2_touch_pos.data(), kfs2_touch_pos.data() + kfs2_touch_pos.size());
            move_group_interface->setJointValueTarget(kfs2_touch_pos_vec);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            count = 0;
            auto error_code = move_group_interface->plan(plan);
            do{
                error_code = move_group_interface->plan(plan);
                count ++ ;
            } while (error_code != moveit::core::MoveItErrorCode::SUCCESS && count < MAX_COUNT);

            if(error_code != moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_ERROR(node->get_logger(), "抓取车上物块路径规划失败");
                return false;
            }

            const double dt = 1.0 / dynamics_params_.control_frequency;
            Eigen::VectorXd current_joint_velocities = Eigen::VectorXd::Zero(6);
            Eigen::VectorXd ee_acceleration = Eigen::VectorXd::Zero(6);
            robot_pose_polynomial_ContinuousTrajectory.set_trajectory(plan.trajectory_.joint_trajectory);
            robot_pose_polynomial_ContinuousTrajectory.start_track(node->now());
            rclcpp::Rate rate(100);
            auto start_time = node->now();
            double trajectory_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + 
                                        plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
            while ((node->now() - start_time).seconds() < trajectory_duration + 1.0){
                Eigen::VectorXd current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(
                    move_group_interface->getCurrentJointValues().data(), move_group_interface->getCurrentJointValues().size());
                Eigen::MatrixXd jacobian = velocity_ik_generator_RobotArmKinematics.computeJacobian(current_joint_positions_);
                trajectory_msgs::msg::JointTrajectoryPoint target_point;
                if(robot_pose_polynomial_ContinuousTrajectory.get_target(node->now(), target_point)){
                    std::vector<double> current_joint_positions = move_group_interface->getCurrentJointValues();
                    std::vector<double> joint_velocities(6);
                    double dt = 0.01;
                    double kp = 100.0;
                    Eigen::VectorXd dynamics_torque = Eigen::VectorXd::Zero(6);
                    bool use_torque_control = false;
                    for(size_t i = 0 ; i < 6 ; ++i){
                        double position_error = target_point.positions[i] - current_joint_positions[i];
                        double feedforward_velocity = target_point.velocities[i];
                        double feedback_velocity = kp * position_error;
                        joint_velocities[i] = feedforward_velocity + feedback_velocity;
                        current_joint_velocities[i] = joint_velocities[i];
                    }

                    if(kdl_dynamics_ && kdl_dynamics_->isInitialized() && dynamics_params_.enable_dynamics_compensation){
                        try{
                            Eigen::VectorXd gravity_compensation = kdl_dynamics_->calculateGravityCompensation(current_joint_positions_);
                            Eigen::VectorXd coriolis_compensation = kdl_dynamics_->calculateCoriolisCompensation(
                                current_joint_positions_, current_joint_velocities);
                            if (!target_point.accelerations.empty()){
                                Eigen::VectorXd joint_acceleration(target_point.accelerations.size());
                                for(size_t i = 0 ; i < target_point.accelerations.size(); ++i){
                                    joint_acceleration[i] = target_point.accelerations[i];
                                }
                                dynamics_torque = kdl_dynamics_->calculateDynamicsTorque(
                                    current_joint_positions_, current_joint_velocities, joint_acceleration);

                                for(size_t i = 0 ; i < 6 ; ++i){
                                    joint_velocities[i] += dynamics_params_.compensation_gain * dynamics_torque(i);
                                }
                            }
                        } catch (const std::exception& e){
                            RCLCPP_WARN(node->get_logger(), "Kinetic compensation failure %s", e.what());
                        }
                    }

                    Eigen::VectorXd joint_velocities_eigen(6);
                    for(size_t i = 0 ; i < 6 ; ++i){
                        joint_velocities_eigen(i) = joint_velocities[i];
                    }

                    send_joint_velocity_to_hardware(joint_velocities_eigen);

                }

                rate.sleep();
            }    

            Eigen::VectorXd zero_velocities(6);
            zero_velocities.setZero();
            send_joint_velocity_to_hardware(zero_velocities);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            return true;
            break;
            }
        case 3:
            {
            return true;
            break;
            }
        default:
            RCLCPP_ERROR(node->get_logger(), "Invalid current_kfs_num: %d", current_kfs_num.load());
            return false;
    }
    
    return true;
}


/**
 * @brief 处理抓取目标状态。
 * 
 * 通过以下方式执行实际抓取操作：
 * 1. 为目标添加碰撞对象
 * 2. 等待机械臂稳定
 * 3. 激活气泵（吸力）
 * 4. 验证气泵状态
 * 5. 添加附加碰撞对象（模拟携带的物体）
 * 6. 验证抓取成功
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::handle_catch_target() {
    RCLCPP_INFO(node->get_logger(), "处理抓取目标状态");
    update_feedback();
    
    // 为目标向规划场景添加碰撞对象
    add_kfs_collision(task_target_pos, "target_kfs", move_group_interface->getPlanningFrame());
    
    // 等待稳定，确保机械臂已到达抓取位置
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 激活气泵并验证
    if(!set_air_pump(true)) {
        RCLCPP_ERROR(node->get_logger(), "激活气泵失败");
        return false;
    }
    
    // 验证气泵状态（如果硬件支持）
    if(!verify_air_pump_status(true)) {
        RCLCPP_WARN(node->get_logger(), "Air pump status verification failed, attempting retry");
        if(!set_air_pump(true)) {
            RCLCPP_ERROR(node->get_logger(), "气泵重新激活失败");
            return false;
        }
    }
    
    // 等待真空建立
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    
    // 添加附加碰撞对象（表示携带的物体）
    if (!add_attached_kfs_collision()) {
        RCLCPP_WARN(node->get_logger(), "附加KFS碰撞体添加失败，附载补偿将不会启用");
        has_attached_kfs_ = false;
    } else {
        has_attached_kfs_ = true;
    }
    
    // 验证抓取是否成功
    if(!verify_grasp_success()) {
        RCLCPP_WARN(node->get_logger(), "抓取验证失败，可能未成功抓取目标");
        // 可以选择重试或继续
    }

    return !cancle_current_task.load();
}


/**
 * @brief 处理移动到释放点状态。
 * 
 * 将携带物体的机器人移动到释放位置。
 * 释放位置取决于任务类型：
 * - 抓取任务：释放到车辆
 * - 放置任务：释放到指定位置
 * 
 * TODO: 实现释放位置计算和路径规划
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::handle_move_to_release_point() {
    RCLCPP_INFO(node->get_logger(), "处理移动到释放位置状态");
    update_feedback();
    
    // ========================================
    // 实现位置3: 移动到释放位置的路径规划
    // ========================================
    // 需要实现的具体内容:
    // 1. 根据KFS数量计算释放位置
    // 2. 对于抓取任务：计算车辆释放位置
    // 3. 对于放置任务：使用任务目标作为释放位置
    // 4. 考虑携带物体时的碰撞检测
    // 5. 实现安全的释放路径规划
    // 6. 添加释放前的位置确认
    
    //**
    // 检索二：KFS 释放到车上的位置
    //*/

    // 当前占位符实现:
    geometry_msgs::msg::Pose release_pose;
    
    if (static_cast<int>(current_task_type.load()) == ArmTask::ROBOTIC_ARM_TASK_CATCH_TARGET) {
        // 抓取任务：释放到车上
        switch (current_kfs_num.load()) {
            case 0:
                {
                // KFS = 0: 释放到车上
                Eigen::VectorXd kfs1_detach_pos(6);
                kfs1_detach_pos << 0.017453293, -0.017453293, 4.101523742, 0.331612558, 0.0, 0.0;
                std::vector<double> kfs1_detach_pos_vec(kfs1_detach_pos.data(), kfs1_detach_pos.data() + kfs1_detach_pos.size());
                move_group_interface->setJointValueTarget(kfs1_detach_pos_vec);

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                moveit::planning_interface::MoveItErrorCode error_code = move_group_interface->plan(plan);
                count = 0;

                do {
                    error_code = move_group_interface->plan(plan);
                    count++;
                } while (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS && count < 100);

                if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(node->get_logger(), "规划到KFS1释放位置失败");
                    return false;
                }

                const double dt = 1.0 / dynamics_params_.control_frequency;


                Eigen::VectorXd current_joint_velocities = Eigen::VectorXd::Zero(6);
                Eigen::VectorXd ee_acceleration = Eigen::VectorXd::Zero(6);
                robot_pose_polynomial_ContinuousTrajectory.set_trajectory(plan.trajectory_.joint_trajectory);
                robot_pose_polynomial_ContinuousTrajectory.start_track(node->now());
                rclcpp::Rate rate(100);
                auto start_time = node->now();
                double trajectory_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + 
                                            plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
                while ((node->now() - start_time).seconds() < trajectory_duration + 1.0){
                    Eigen::VectorXd current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(
                        move_group_interface->getCurrentJointValues().data(), move_group_interface->getCurrentJointValues().size());
                    Eigen::MatrixXd jacobian = velocity_ik_generator_RobotArmKinematics.computeJacobian(current_joint_positions_);
                    trajectory_msgs::msg::JointTrajectoryPoint target_point;
                    if(robot_pose_polynomial_ContinuousTrajectory.get_target(node->now(), target_point)){
                        std::vector<double> current_joint_positions = move_group_interface->getCurrentJointValues();
                        std::vector<double> joint_velocities(6);
                        double dt = 0.01;
                        double kp = 100.0;
                        Eigen::VectorXd dynamics_torque = Eigen::VectorXd::Zero(6);
                        bool use_torque_control = false;
                        for(size_t i = 0 ; i < 6 ; ++i){
                            double position_error = target_point.positions[i] - current_joint_positions[i];
                            double feedforward_velocity = target_point.velocities[i];
                            double feedback_velocity = kp * position_error;
                            joint_velocities[i] = feedforward_velocity + feedback_velocity;
                            current_joint_velocities[i] = joint_velocities[i];
                        }

                        if(kdl_dynamics_ && kdl_dynamics_->isInitialized() && dynamics_params_.enable_dynamics_compensation){
                            try{
                                Eigen::VectorXd gravity_compensation = kdl_dynamics_->calculateGravityCompensation(current_joint_positions_);
                                Eigen::VectorXd coriolis_compensation = kdl_dynamics_->calculateCoriolisCompensation(
                                    current_joint_positions_, current_joint_velocities);

                                
                                if (!target_point.accelerations.empty()){
                                    Eigen::VectorXd joint_acceleration(target_point.accelerations.size());
                                    for(size_t i = 0 ; i < target_point.accelerations.size(); ++i){
                                        joint_acceleration[i] = target_point.accelerations[i];
                                    }
                                    dynamics_torque = kdl_dynamics_->calculateDynamicsTorque(
                                        current_joint_positions_, current_joint_velocities, joint_acceleration);
                                    dynamics_torque += calculate_kfs_payload_compensation(current_joint_positions_);

                                    for(size_t i = 0 ; i < 6 ; ++i){
                                        joint_velocities[i] += dynamics_params_.compensation_gain * dynamics_torque(i);
                                    }
                                }
                            } catch (const std::exception& e){
                                RCLCPP_WARN(node->get_logger(), "Kinetic compensation failure %s", e.what());
                            }
                        }

                        Eigen::VectorXd joint_velocities_eigen(6);
                        for(size_t i = 0 ; i < 6 ; ++i){
                            joint_velocities_eigen(i) = joint_velocities[i];
                        }

                        send_joint_velocity_to_hardware(joint_velocities_eigen);

                    }

                    rate.sleep();
                }    

                Eigen::VectorXd zero_velocities(6);
                zero_velocities.setZero();
                send_joint_velocity_to_hardware(zero_velocities);

                

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                return true;

                break;
                }
            case 1:
                {
                // KFS = 1: 释放到车上
                Eigen::VectorXd kfs2_detach_pos(6);
                kfs2_detach_pos << -0.087266463, 0.087266463, 3.420845333, -0.418879020, 0.0, 0.0;
                std::vector<double> kfs2_detach_pos_vec(kfs2_detach_pos.data(), kfs2_detach_pos.data() + kfs2_detach_pos.size());
                move_group_interface->setJointValueTarget(kfs2_detach_pos_vec);

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                moveit::planning_interface::MoveItErrorCode error_code = move_group_interface->plan(plan);
                count = 0;

                do {
                    error_code = move_group_interface->plan(plan);
                    count++;
                } while (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS && count < 100);

                if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(node->get_logger(), "规划到KFS2释放位置失败");
                    return false;
                }

                const double dt = 1.0 / dynamics_params_.control_frequency;


                Eigen::VectorXd current_joint_velocities = Eigen::VectorXd::Zero(6);
                Eigen::VectorXd ee_acceleration = Eigen::VectorXd::Zero(6);
                robot_pose_polynomial_ContinuousTrajectory.set_trajectory(plan.trajectory_.joint_trajectory);
                robot_pose_polynomial_ContinuousTrajectory.start_track(node->now());
                rclcpp::Rate rate(100);
                auto start_time = node->now();
                double trajectory_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + 
                                            plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
                while ((node->now() - start_time).seconds() < trajectory_duration + 1.0){
                    Eigen::VectorXd current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(
                        move_group_interface->getCurrentJointValues().data(), move_group_interface->getCurrentJointValues().size());
                    Eigen::MatrixXd jacobian = velocity_ik_generator_RobotArmKinematics.computeJacobian(current_joint_positions_);
                    trajectory_msgs::msg::JointTrajectoryPoint target_point;
                    if(robot_pose_polynomial_ContinuousTrajectory.get_target(node->now(), target_point)){
                        std::vector<double> current_joint_positions = move_group_interface->getCurrentJointValues();
                        std::vector<double> joint_velocities(6);
                        double dt = 0.01;
                        double kp = 100.0;
                        Eigen::VectorXd dynamics_torque = Eigen::VectorXd::Zero(6);
                        bool use_torque_control = false;
                        for(size_t i = 0 ; i < 6 ; ++i){
                            double position_error = target_point.positions[i] - current_joint_positions[i];
                            double feedforward_velocity = target_point.velocities[i];
                            double feedback_velocity = kp * position_error;
                            joint_velocities[i] = feedforward_velocity + feedback_velocity;
                            current_joint_velocities[i] = joint_velocities[i];
                        }

                        if(kdl_dynamics_ && kdl_dynamics_->isInitialized() && dynamics_params_.enable_dynamics_compensation){
                            try{
                                Eigen::VectorXd gravity_compensation = kdl_dynamics_->calculateGravityCompensation(current_joint_positions_);
                                Eigen::VectorXd coriolis_compensation = kdl_dynamics_->calculateCoriolisCompensation(
                                    current_joint_positions_, current_joint_velocities);

                                
                                if (!target_point.accelerations.empty()){
                                    Eigen::VectorXd joint_acceleration(target_point.accelerations.size());
                                    for(size_t i = 0 ; i < target_point.accelerations.size(); ++i){
                                        joint_acceleration[i] = target_point.accelerations[i];
                                    }
                                    dynamics_torque = kdl_dynamics_->calculateDynamicsTorque(
                                        current_joint_positions_, current_joint_velocities, joint_acceleration);
                                    dynamics_torque += calculate_kfs_payload_compensation(current_joint_positions_);

                                    for(size_t i = 0 ; i < 6 ; ++i){
                                        joint_velocities[i] += dynamics_params_.compensation_gain * dynamics_torque(i);
                                    }
                                }
                            } catch (const std::exception& e){
                                RCLCPP_WARN(node->get_logger(), "Kinetic compensation failure %s", e.what());
                            }
                        }

                        Eigen::VectorXd joint_velocities_eigen(6);
                        for(size_t i = 0 ; i < 6 ; ++i){
                            joint_velocities_eigen(i) = joint_velocities[i];
                        }

                        send_joint_velocity_to_hardware(joint_velocities_eigen);

                    }

                    rate.sleep();
                }    

                Eigen::VectorXd zero_velocities(6);
                zero_velocities.setZero();
                send_joint_velocity_to_hardware(zero_velocities);

                

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                return true;

                break;
                }
            case 2:
                {
                // KFS = 2: 释放到车上
                Eigen::VectorXd kfs3_hold_pos(6);
                kfs3_hold_pos << -0.052359878, 0.087266463, 3.595305762, -2.111848394, -0.052359878, 0.0;
                std::vector<double> kfs3_hold_pos_vec(kfs3_hold_pos.data(), kfs3_hold_pos.data() + kfs3_hold_pos.size());
                move_group_interface->setJointValueTarget(kfs3_hold_pos_vec);

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                moveit::planning_interface::MoveItErrorCode error_code = move_group_interface->plan(plan);
                count = 0;

                do {
                    error_code = move_group_interface->plan(plan);
                    count++;
                } while (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS && count < 100);

                if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(node->get_logger(), "规划到KFS3释放位置失败");
                    return false;
                }

                const double dt = 1.0 / dynamics_params_.control_frequency;

                Eigen::VectorXd current_joint_velocities = Eigen::VectorXd::Zero(6);
                Eigen::VectorXd ee_acceleration = Eigen::VectorXd::Zero(6);
                robot_pose_polynomial_ContinuousTrajectory.set_trajectory(plan.trajectory_.joint_trajectory);
                robot_pose_polynomial_ContinuousTrajectory.start_track(node->now());
                rclcpp::Rate rate(100);
                auto start_time = node->now();
                double trajectory_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + 
                                            plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
                while ((node->now() - start_time).seconds() < trajectory_duration + 1.0){
                    Eigen::VectorXd current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(
                        move_group_interface->getCurrentJointValues().data(), move_group_interface->getCurrentJointValues().size());
                    Eigen::MatrixXd jacobian = velocity_ik_generator_RobotArmKinematics.computeJacobian(current_joint_positions_);
                    trajectory_msgs::msg::JointTrajectoryPoint target_point;

                    if(robot_pose_polynomial_ContinuousTrajectory.get_target(node->now(), target_point)){
                        std::vector<double> current_joint_positions = move_group_interface->getCurrentJointValues();
                        std::vector<double> joint_velocities(6);
                        double dt = 0.01;
                        double kp = 100.0;
                        Eigen::VectorXd dynamics_torque = Eigen::VectorXd::Zero(6);
                        bool use_torque_control = false;
                        for(size_t i = 0 ; i < 6 ; ++i){
                            double position_error = target_point.positions[i] - current_joint_positions[i];
                            double feedforward_velocity = target_point.velocities[i];
                            double feedback_velocity = kp * position_error;
                            joint_velocities[i] = feedforward_velocity + feedback_velocity;
                            current_joint_velocities[i] = joint_velocities[i];
                        }

                        if(kdl_dynamics_ && kdl_dynamics_->isInitialized() && dynamics_params_.enable_dynamics_compensation){
                            try{
                                Eigen::VectorXd gravity_compensation = kdl_dynamics_->calculateGravityCompensation(current_joint_positions_);
                                Eigen::VectorXd coriolis_compensation = kdl_dynamics_->calculateCoriolisCompensation(
                                    current_joint_positions_, current_joint_velocities);
                                
                                if (!target_point.accelerations.empty()){
                                    Eigen::VectorXd joint_acceleration(target_point.accelerations.size());
                                    for(size_t i = 0 ; i < target_point.accelerations.size(); ++i){
                                        joint_acceleration[i] = target_point.accelerations[i];
                                    }
                                    dynamics_torque = kdl_dynamics_->calculateDynamicsTorque(
                                        current_joint_positions_, current_joint_velocities, joint_acceleration);
                                    dynamics_torque += calculate_kfs_payload_compensation(current_joint_positions_);

                                    for(size_t i = 0 ; i < 6 ; ++i){
                                        joint_velocities[i] += dynamics_params_.compensation_gain * dynamics_torque(i);
                                    }
                                }
                            } catch (const std::exception& e){
                                RCLCPP_WARN(node->get_logger(), "Kinetic compensation failure %s", e.what());
                            }
                        }

                        Eigen::VectorXd joint_velocities_eigen(6);
                        for(size_t i = 0 ; i < 6 ; ++i){
                            joint_velocities_eigen(i) = joint_velocities[i];
                        }

                        send_joint_velocity_to_hardware(joint_velocities_eigen);

                    }

                    rate.sleep();
                }    

                Eigen::VectorXd zero_velocities(6);
                zero_velocities.setZero();
                send_joint_velocity_to_hardware(zero_velocities);

                

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                return true;
                break;
                }
            case 3:
                {
                // KFS = 3: 释放到车上
                RCLCPP_ERROR(node->get_logger(), "KFS 已满");
                return false;
                }
            default:
                break;
        }
    } else if (static_cast<int>(current_task_type.load()) == ArmTask::ROBOTIC_ARM_TASK_PLACE_TARGET) {
        // 放置任务：释放到指定位置
        // TODO: 使用任务目标作为释放位置
        //**
        // 检索七：架子上的释放位置
        //*/
        release_pose = task_target_pos;
        move_group_interface->setPoseTarget(release_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode error_code = move_group_interface->plan(plan);
        count = 0;

        do{
            error_code = move_group_interface->plan(plan);
            count += 1;
        } while (!error_code == moveit::core::MoveItErrorCode::SUCCESS && count < MAX_COUNT);

        if(error_code != moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_WARN(node->get_logger(), "Failed to plan path for place target");
            return false;
        }

        const double dt = 1.0 / dynamics_params_.control_frequency;
        Eigen::VectorXd current_joint_velocities = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd ee_acceleration = Eigen::VectorXd::Zero(6);
        robot_pose_polynomial_ContinuousTrajectory.set_trajectory(plan.trajectory_.joint_trajectory);
        robot_pose_polynomial_ContinuousTrajectory.start_track(node->now());
        rclcpp::Rate rate(100);

        auto start_time = node->now();
        double trajectory_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + 
                                     plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec / 1e9;

        while ((node->now() - start_time).seconds() < trajectory_duration + 1.0){
            Eigen::VectorXd current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(
                move_group_interface->getCurrentJointValues().data(), move_group_interface->getCurrentJointValues().size());

            Eigen::MatrixXd jacobian = velocity_ik_generator_RobotArmKinematics.computeJacobian(current_joint_positions_);

            trajectory_msgs::msg::JointTrajectoryPoint target_point;
            if(robot_pose_polynomial_ContinuousTrajectory.get_target(node->now(), target_point)){
                std::vector<double> current_joint_positions = move_group_interface->getCurrentJointValues();
                std::vector<double> joint_velocities(6);
                
                double dt = 0.01;
                double kp = 100.0;
                Eigen::VectorXd dynamics_torque = Eigen::VectorXd::Zero(6);
                bool use_torque_control = false;
                for(size_t i = 0 ; i < 6 ; ++i){
                    double position_error = target_point.positions[i] - current_joint_positions[i];
                    double feedforward_velocity = target_point.velocities[i];
                    double feedback_velocity = kp * position_error;
                    joint_velocities[i] = feedforward_velocity + feedback_velocity;
                    current_joint_velocities[i] = joint_velocities[i];
                }

                if(kdl_dynamics_ && kdl_dynamics_->isInitialized() && dynamics_params_.enable_dynamics_compensation){
                    try{
                        Eigen::VectorXd gravity_compensation = kdl_dynamics_->calculateGravityCompensation(current_joint_positions_);
                        Eigen::VectorXd coriolis_compensation = kdl_dynamics_->calculateCoriolisCompensation(
                            current_joint_positions_, current_joint_velocities);

                        if (!target_point.accelerations.empty()){
                            Eigen::VectorXd joint_acceleration(target_point.accelerations.size());

                            for (size_t i = 0 ; i < target_point.accelerations.size() ; ++i){
                                joint_acceleration[i] = target_point.accelerations[i];
                            }

                            dynamics_torque = kdl_dynamics_->calculateDynamicsTorque(
                                current_joint_positions_, current_joint_velocities, joint_acceleration);

                            dynamics_torque += calculate_kfs_payload_compensation(current_joint_positions_);

                            for(size_t i = 0 ; i < 6 ; ++i){
                                joint_velocities[i] = dynamics_params_.compensation_gain * dynamics_torque(i);
                            }
                        }
                    } catch (const std::exception& e){
                        RCLCPP_WARN(node->get_logger(), "Kinetic compensation failure %s", e.what());
                    }

                    Eigen::VectorXd joint_velocities_eigen(6);
                    for(size_t i = 0 ; i < 6 ; ++i){
                        joint_velocities_eigen(i) = joint_velocities[i];
                    }

                    send_joint_velocity_to_hardware(joint_velocities_eigen);
                }

                rate.sleep();
            }

            Eigen::VectorXd zero_velocities(6);
            zero_velocities.setZero();
            send_joint_velocity_to_hardware(zero_velocities);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        }
    }
    
    // TODO: 添加释放位置路径规划逻辑
    // 考虑携带物体时的动力学约束
    // 添加释放前的最终位置确认
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    return false;
}


/**
 * @brief 处理释放目标状态。
 * 
 * 通过以下方式执行释放操作：
 * 1. 停用气泵（释放吸力）
 * 2. 验证气泵已关闭
 * 3. 等待完全释放
 * 4. 移除附加碰撞对象
 * 5. 移除目标碰撞对象
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::handle_release_target() {
    RCLCPP_INFO(node->get_logger(), "处理释放目标状态");
    update_feedback();
    
    // 停用气泵
    if(!set_air_pump(false)) {
        RCLCPP_ERROR(node->get_logger(), "停用气泵失败");
        return false;
    }
    
    // 验证气泵确实已关闭
    if(!verify_air_pump_status(false)) {
        RCLCPP_WARN(node->get_logger(), "Air pump status verification failed, attempting retry");
        if(!set_air_pump(false)) {
            RCLCPP_ERROR(node->get_logger(), "气泵重新停用失败");
            return false;
        }
    }
    
    // 等待完全释放
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // Remove attached collision object
    remove_attached_kfs_collision();
    has_attached_kfs_ = false;
    
    // 移除碰撞对象
    remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    return !cancle_current_task.load();
}


/**
 * @brief 处理移动到空闲点状态。
 * 
 * 将机器人移动到任务完成后的安全空闲位置。
 * 
 * TODO: 实现安全的空闲位置计算和路径规划
 * 
 * @return 如果成功返回true，否则返回false
 */
bool RoboticTask::handle_move_to_idle_point() {
    RCLCPP_INFO(node->get_logger(), "处理移动到空闲位置状态");
    update_feedback();
    
    // ========================================
    // 实现位置4: 移动到空闲位置的路径规划
    // ========================================
    // 需要实现的具体内容:
    // 1. 定义安全的空闲位置（通常为机械臂的初始位置）
    // 2. 设置所有关节角度到安全位置
    // 3. 或设置末端执行器到安全坐标
    // 4. 确保路径无碰撞
    // 5. 实现平滑的归位运动
    // 6. 添加到达确认
    
    // 当前占位符实现:
    // 定义空闲位置（可根据实际机械臂调整）

    //**
    // 检索三：运行中的空闲位置
    //*/

    Eigen::VectorXd idle_joint_positions(6);
    idle_joint_positions << 0.0, 1.064650844, 0.087266463, -1.989675347, 0.0, 0.0;
    std::vector<double> idle_joint_values(idle_joint_positions.data(), idle_joint_positions.data() + idle_joint_positions.size());
    move_group_interface->setJointValueTarget(idle_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit::planning_interface::MoveItErrorCode error_code = move_group_interface->plan(plan);
    count = 0 ;
    do{
        error_code = move_group_interface->plan(plan);
        count++;
    } while (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS && count < MAX_COUNT);

    if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS){
        RCLCPP_ERROR(node->get_logger(), "运行中的空闲位置失败");
        return false;
    }

    const double dt = 1.0 / dynamics_params_.control_frequency;
    Eigen::VectorXd current_joint_velocities = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ee_acceleration = Eigen::VectorXd::Zero(6);
    robot_pose_polynomial_ContinuousTrajectory.set_trajectory(plan.trajectory_.joint_trajectory);
    robot_pose_polynomial_ContinuousTrajectory.start_track(node->now());
    rclcpp::Rate rate(100);
    auto start_time = node->now();
    double trajectory_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + 
                                plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
    while ((node->now() - start_time).seconds() < trajectory_duration + 1.0){
        Eigen::VectorXd current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(
            move_group_interface->getCurrentJointValues().data(), move_group_interface->getCurrentJointValues().size());
        Eigen::MatrixXd jacobian = velocity_ik_generator_RobotArmKinematics.computeJacobian(current_joint_positions_);
        trajectory_msgs::msg::JointTrajectoryPoint target_point;
        if(robot_pose_polynomial_ContinuousTrajectory.get_target(node->now(), target_point)){
            std::vector<double> current_joint_positions = move_group_interface->getCurrentJointValues();
            std::vector<double> joint_velocities(6);
            double dt = 0.01;
            double kp = 100.0;
            Eigen::VectorXd dynamics_torque = Eigen::VectorXd::Zero(6);
            bool use_torque_control = false;
            for(size_t i = 0 ; i < 6 ; ++i){
                double position_error = target_point.positions[i] - current_joint_positions[i];
                double feedforward_velocity = target_point.velocities[i];
                double feedback_velocity = kp * position_error;
                joint_velocities[i] = feedforward_velocity + feedback_velocity;
                current_joint_velocities[i] = joint_velocities[i];
            }

            if(kdl_dynamics_ && kdl_dynamics_->isInitialized() && dynamics_params_.enable_dynamics_compensation){
                try{
                    Eigen::VectorXd gravity_compensation = kdl_dynamics_->calculateGravityCompensation(current_joint_positions_);
                    Eigen::VectorXd coriolis_compensation = kdl_dynamics_->calculateCoriolisCompensation(
                        current_joint_positions_, current_joint_velocities);
                    if (!target_point.accelerations.empty()){
                        Eigen::VectorXd joint_acceleration(target_point.accelerations.size());
                        for(size_t i = 0 ; i < target_point.accelerations.size(); ++i){
                            joint_acceleration[i] = target_point.accelerations[i];
                        }
                        dynamics_torque = kdl_dynamics_->calculateDynamicsTorque(
                            current_joint_positions_, current_joint_velocities, joint_acceleration);

                        for(size_t i = 0 ; i < 6 ; ++i){
                            joint_velocities[i] += dynamics_params_.compensation_gain * dynamics_torque(i);
                        }
                    }
                } catch (const std::exception& e){
                    RCLCPP_WARN(node->get_logger(), "Kinetic compensation failure %s", e.what());
                }
            }

            Eigen::VectorXd joint_velocities_eigen(6);
            for(size_t i = 0 ; i < 6 ; ++i){
                joint_velocities_eigen(i) = joint_velocities[i];
            }

            send_joint_velocity_to_hardware(joint_velocities_eigen);

        }

        rate.sleep();
    }    

    Eigen::VectorXd zero_velocities(6);
    zero_velocities.setZero();
    send_joint_velocity_to_hardware(zero_velocities);

    
    // TODO: 执行路径规划并执行
    // 确保所有关节都在安全范围内
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    return !cancle_current_task.load();
}


// 状态机工具函数

/**
 * @brief 转换到新状态。
 * 
 * 更新当前状态并记录转换用于调试。
 * 
 * @param new_state 要转换到的新状态
 */
void RoboticTask::transition_to_state(ArmTaskState new_state) {
    current_state.store(new_state);
    RCLCPP_INFO(node->get_logger(), "状态转换到: %s", get_state_description(new_state).c_str());
}


/**
 * @brief 使用当前状态更新动作反馈。
 * 
 * 向动作客户端发送包含当前状态
 * 和描述的反馈，以保持客户端了解进度。
 */
void RoboticTask::update_feedback() {
    if(current_goal_handle) {
        auto feedback_msg = std::make_shared<robot_interfaces::action::Catch::Feedback>();
        feedback_msg->current_state = current_state.load();
        feedback_msg->state_describe = get_state_description(static_cast<ArmTaskState>(current_state.load()));
        current_goal_handle->publish_feedback(feedback_msg);
    }
}


/**
 * @brief 将任务状态重置为初始条件。
 * 
 * 重置所有任务相关变量和标志，为
 * 下一个任务执行做准备。
 */
void RoboticTask::reset_task_state() {
    RCLCPP_INFO(node->get_logger(), "重置任务状态");
    current_state.store(ArmTaskState::ROBOTIC_ARM_TASK_STATE_IDLE);
    cancle_current_task.store(false);
    current_goal_handle.reset();
    
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        is_running_arm_task = false;
    }
}

/**
 * @brief 构建末端速度向量
 * 
 * 末端线速度和与目标距离成正比
 * 末端角速度为末端角速度为0
 *
 * TODO:更好的速度计算函数
 *
 */
void RoboticTask::calculate_end_effector_velocity(
    double x, 
    double y,
    double z
){
    velocity_ik_generator_CartesianTwist.linear = Eigen::Vector3d(
        std::min(x, MAX_END_EFFECTOR_VELOCITY), 
        std::min(y, MAX_END_EFFECTOR_VELOCITY), 
        std::min(z, MAX_END_EFFECTOR_VELOCITY)
    );
    velocity_ik_generator_CartesianTwist.angular = Eigen::Vector3d::Zero();
    end_effector_velocity.x() = velocity_ik_generator_CartesianTwist.linear.x();
    end_effector_velocity.y() = velocity_ik_generator_CartesianTwist.linear.y();
    end_effector_velocity.z() = velocity_ik_generator_CartesianTwist.linear.z();
}

/**
 * @brief 由末端速度计算关节速度
 * 
 * 调用jointStateCallback获取最新joint_position
 * 调用 velocity_ik_generator_RobotArmKinematics 中的 inverseVelocityKinematics
 * 
 */
Eigen::VectorXd RoboticTask::calculate_joint_velocity(
    const Eigen::Vector3d& end_effector_velocity
) const {
    // 获取当前关节位置
    Eigen::VectorXd joint_positions = get_joint_position();
    
    // 构造末端速度Twist（只有线速度，角速度为零）
    RobotKinematicsKDL::CartesianTwist desired_twist;
    desired_twist.linear = end_effector_velocity;
    desired_twist.angular = Eigen::Vector3d::Zero();
    
    // 调用逆速度运动学计算关节速度
    return velocity_ik_generator_RobotArmKinematics.inverseVelocityKinematics(
        joint_positions, 
        desired_twist
    );
}



/**
 * @brief 获取状态的描述字符串。
 * 
 * 将状态枚举值转换为人类可读的字符串
 * 用于记录和调试目的。
 * 
 * @param state 要获取描述的状态
 * @return 状态的字符串描述
 */
std::string RoboticTask::get_state_description(ArmTaskState state) {
    switch(state) {
        case ArmTaskState::ROBOTIC_ARM_TASK_STATE_IDLE:
            return "空闲状态";
        case ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_READY_CATCH_POINT:
            return "移动到预备抓取位置";
        case ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_CATCH_POINT:
            return "移动到抓取位置";
        case ArmTaskState::ROBOTIC_ARM_TASK_STATE_CATCH_TARGET:
            return "抓取目标";
        case ArmTaskState::ROBOTIC_ARM_TASK_STATE_VISUAL_SERVOING:
            return "视觉伺服";
        case ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_RELEASE_POINT:
            return "移动到释放位置";
        case ArmTaskState::ROBOTIC_ARM_TASK_STATE_RELEASE_TARGET:
            return "释放目标";
        case ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_IDLE_POINT:
            return "移动到空闲位置";
        case ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_READY_CATCH_POINT_KFS_NOT_ZERO:
            return "移动到预备抓取位置（KFS != 0）";
        case ArmTaskState::ROBOTIC_ARM_TASK_STATE_MOVE_TO_RELEASE_POINT_IN_SHELF:
            return "移动到释放位置（将kfs放到架子上）";
        default:
            return "未知状态";
    }
}

bool RoboticTask::checkJointTorqueLimits(const Eigen::VectorXd& joint_torques) const {
    if (joint_torques.size() != 6) {
        RCLCPP_ERROR(node->get_logger(), "关节力矩向量维度错误");
        return false;
    }
    
    for (int i = 0; i < joint_torques.size(); ++i) {
        if (std::abs(joint_torques(i)) > dynamics_params_.max_joint_torque) {
            RCLCPP_WARN(node->get_logger(), "关节 %d 力矩 %.2f 超出限制 %.2f", 
                       i, joint_torques(i), dynamics_params_.max_joint_torque);
            return false;
        }
    }
    return true;
}

bool RoboticTask::checkEmergencyStop(
    const Eigen::VectorXd& joint_positions,
    const Eigen::VectorXd& joint_velocities,
    const Eigen::VectorXd& joint_accelerations
) const {
    // 检查关节速度是否过大
    for (int i = 0; i < joint_velocities.size(); ++i) {
        if (std::abs(joint_velocities(i)) > dynamics_params_.emergency_stop_threshold) {
            RCLCPP_ERROR(node->get_logger(), "关节 %d 速度 %.2f 超出紧急停止阈值 %.2f", 
                        i, joint_velocities(i), dynamics_params_.emergency_stop_threshold);
            return true;
        }
    }
    
    // 检查关节加速度是否过大
    for (int i = 0; i < joint_accelerations.size(); ++i) {
        if (std::abs(joint_accelerations(i)) > dynamics_params_.max_joint_acceleration * 5.0) {
            RCLCPP_ERROR(node->get_logger(), "关节 %d 加速度 %.2f 过大", 
                        i, joint_accelerations(i));
            return true;
        }
    }
    
    // 检查关节位置是否在合理范围内（可以根据实际机器人调整）
    for (int i = 0; i < joint_positions.size(); ++i) {
        if (std::abs(joint_positions(i)) > M_PI) {
            RCLCPP_ERROR(node->get_logger(), "关节 %d 位置 %.2f 超出合理范围", 
                        i, joint_positions(i));
            return true;
        }
    }
    
    return false;
}

Eigen::VectorXd RoboticTask::calculate_kfs_payload_compensation(
    const Eigen::VectorXd& joint_positions
) const {
    if (!has_attached_kfs_ || !kdl_dynamics_ || !kdl_dynamics_->isInitialized()) {
        return Eigen::VectorXd::Zero(6);
    }

    //**
    // TODO: 从配置中读取负载质量
    //  检索五：KFS质量
    //*/
    double payload_mass = 0.5; // 假设负载质量为0.5kg

    //**
    // TODO: KFS坐标偏移
    // 检索六：KFS坐标偏移 
    //*/
    Eigen::Vector3d payload_com_in_ee = Eigen::Vector3d(0.0, 0.0, -0.175);
    try {
        return kdl_dynamics_->calculatePayloadGravityCompensation(
            joint_positions,
            dynamics_params_.kfs_payload_mass,
            kfs_payload_com_in_ee_
        );
    } catch (const std::exception& e) {
        RCLCPP_WARN(node->get_logger(), "KFS附载重力补偿计算失败: %s", e.what());
        return Eigen::VectorXd::Zero(6);
    }
}

void RoboticTask::updateDynamicsParams(const DynamicsControlParams& params) {
    dynamics_params_ = params;
    RCLCPP_INFO(node->get_logger(), "动力学控制参数已更新");
}












