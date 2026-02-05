#include "robotic_task.hpp"
#include "robot_pose_polynomial.hpp"
#include "velocity_ik_generator.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cassert>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/detail/constraints__struct.hpp>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/utilities.hpp>
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

RoboticTask::RoboticTask(const rclcpp::Node::SharedPtr node) : node(node){
    // 初始化关节位置为6维零向量（假设6关节机器人）
    joint_position = Eigen::VectorXd::Zero(6);
    
    param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "driver_node");
    arm_handle_server = rclcpp::action::create_server<robot_interafces::action::Catch>(node, "robotic_task", 
        std::bind(&RoboticTask::handle_goal, this, std::placeholders::_1, std::placeholders::_2), 
        std::bind(&RoboticTask::cancel_goal, this, std::placeholders::_1), 
        std::bind(&RoboticTask::handle_accepted, this, std::placeholders::_1)
    );

    camera_link0_tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    camera_link0_tf_lisenter_ = std::make_shared<tf2_ros::TransformListener>(*camera_link0_tf_buffer);
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robotic_arm");
    psi = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    mark_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("debug_marker", 10);

    joint_state_subscriber_ = node->create_subscription<robot_interfaces::msg::Robot>(
        
    )

    node->create_wall_timer
    (
        100ms, 
        [this](){
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            if(!is_running_arm_task) return;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->node->now();
        marker.ns = "kfs_pos";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;  // 显示球体标记
        marker.action = visualization_msgs::msg::Marker::ADD;   // 增加标记
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

    attached_kfs_pos.orientation.w = 1.0;
    attached_kfs_pos.position.x = 0.0;
    attached_kfs_pos.position.y = 0.0;
    attached_kfs_pos.position.z = -0.24;

    move_group_interface->setPlanningTime(10.0);
    move_group_interface->setMaxVelocityScalingFactor(VELOCITY_SCALING);            // 设置速度缩放因子
    move_group_interface->setMaxAccelerationScalingFactor(ACCELERATION_SCALING);    // 设置加速度缩放因子

    // move_group_interface->allowReplanning(true); // 允许重新规划
    // move_group_interface->setPlannerId("RRTConnectkConfigDefault"); // 设置规划器
    // move_group_interface->setNumPlanningAttempts(10); // 设置规划尝试次数

    // move_group_interface->setGoalPositionTolerance(0.01); // 设置目标位置容差
    // move_group_interface->setGoalOrientationTolerance(0.01); // 设置目标方向容差
    // move_group_interface->setPoseReferenceFrame("base_link"); // 设置目标参考坐标系
    // move_group_interface->setEndEffectorLink("wrist_3_link"); // 设置末端效果器链接
    // move_group_interface->setSupportSurfaceName("table"); // 设置支持表面名称

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());  // 创建 TF2 缓存
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // 创建 TF2 监听器

    try{
        arm_task_thread = std::make_unique<std::thread>([this](){arm_catch_task_handle();});
    } catch (const std::exception& e){
        RCLCPP_ERROR(node->get_logger(), "创建机械臂任务线程失败： %s", e.what());
        arm_task_thread = nullptr;
    }
}


RoboticTask::~RoboticTask(){
    task_mutex_.lock();  // 加锁
    has_new_task_ = true;
    task_mutex_.unlock();
    task_cv_.notify_one(); 
    
    if(arm_task_thread && arm_task_thread->joinable()){
        arm_task_thread->join();
    }
}

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

    try{
        camera_link0_tf = camera_link0_tf_buffer->lookupTransform("base_link", "camera_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex){
        RCLCPP_WARN(node->get_logger(), "无法获取相机到基座的变换： %s", ex.what());
        return rclcpp_action::GoalResponse::REJECT;
    }

    tf2::doTransform(goal->target_pose, task_target_pos, camera_link0_tf);
    RCLCPP_INFO(node->get_logger(), "原始目标位姿： Pos(%lf, %lf, %lf), Ori(%lf, %lf, %lf, %lf)",
                goal->target_pose.position.x, goal->target_pose.position.y, 
                goal->target_pose.position.z, goal->target_pose.orientation.x, 
                goal->target_pose.orientation.y, goal->target_pose.orientation.z, 
                goal->target_pose.orientation.w);

    auto qin = task_target_pos.orientation;
    tf2::Quaternion q(qin.x, qin.y, qin.z, qin.w);
    q.normalize();
    task_target_pos.orientation.x = q.x();
    task_target_pos.orientation.y = q.y();
    task_target_pos.orientation.z = q.z();
    task_target_pos.orientation.w = q.w();

    current_task_type = goal->action_type; // 设置当前任务类型

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RoboticTask::cancel_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>>& goal_handle 
){
    (void)goal_handle;
    cancel_current_task = true;
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        is_running_arm_task = false;
    }

    remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame());

    set_air_pump(false);

    return rclcpp_action::CancelResponse::ACCEPT;

}
























geometry_msgs::msg::Pose RoboticTask::calculate_target_pose(
    const geometry_msgs::msg::Pose& box_pos, 
    double approach_distance, 
    geometry_msgs::msg::Pose& grasp_pose, 
    ApproachMode mode 
){
    RCLCPP_INFO(
        node->get_logger(),"传入的box_pos: Pos(%lf, %lf, %lf), Ori(%lf, %lf, %lf, %lf)", 
        box_pos.position.x, box_pos.position.y, box_pos.position.z, 
        box_pos.orientation.x, box_pos.orientation.y, box_pos.orientation.z, box_pos.orientation.w
    );
    
    Eigen::Vector3d object_center(box_pos.position.x, box_pos.position.y, box_pos.position.z);
    Eigen::Quaterniond object_quat(box_pos.orientation.w, box_pos.orientation.x, box_pos.orientation.y, box_pos.orientation.z);
    Eigen::Matrix3d object_rot = object_quat.toRotationMatrix(); // 转换为旋转矩阵

    Eigen::Vector3d surface_normal = R * Eigen::Vector3d(0.0, 0.0, 1.0);
    RCLCPP_INFO(node->get_logger(), "表面法线 = (%f, %f, %f)", surface_normal.x(), surface_normal.y(), surface_normal.z());
    
    Eigen::Vector3d to_robot_base = -object_center;
    
    if(surface_normal.dot(to_robot_base) < 0.0){  // dot 计算点积
        surface_normal = -surface_normal;
        RCLCPP_INFO(node->get_logger(), "反转表面法线为 (%f, %f, %f)", surface_normal.x(), surface_normal.y(), surface_normal.z());
    }

    const double object_half_size = 0.175;
    Eigen::Vector3d surface_position = object_center + object_half_size * surface_normal;
    RCLCPP_INFO(node->get_logger(), "表面位置 = (%f, %f, %f)", surface_position.x(), surface_position.y(), surface_position.z());

    const double inside_offset = 0.05;
    const double outside_offset = 0.05;

    Eigen::Vector3d grasp_position = surface_position - inside_offset * surface_normal;
    Eigen::Vector3d prepare_position = surface_position + outside_offset * surface_normal;

    RCLCPP_INFO(node->get_logger(), "抓取位置 = (%f, %f, %f)", grasp_position.x(), grasp_position.y(), grasp_position.z());
    RCLCPP_INFO(node->get_logger(), "准备位置 = (%f, %f, %f)", prepare_position.x(), prepare_position.y(), prepare_position.z());

    Eigen::Vector3d robot_base(0.0, 0.0, 0.0);
    Eigen::Vector3d robot_to_object = object_center - robot_base;
    Eigen::Vector3d  robot_side_direction;
    robot_side_direction = robot_to_object;
    robot_side_direction.z() = 0;
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

    Eigen::Vector3d eef_x_axis;
    Eigen::Vector3d eef_y_axis;
    Eigen::Vector3d eef_z_axis;

    Eigen::Vector3d suction_dir = away_from_robot - away_from_robot.dot(surface_normal) * surface_normal; // 计算吸盘方向
    suction_dir.normalize();

    RCLCPP_INFO(
        node->get_logger(), 
        "吸盘方向计算 = (%f, %f, %f)", suction_dir.x(), suction_dir.y(), suction_dir.z()
    );

    double normal_alignment = std::abs(suction_dir.dot(surface_normal));
    RCLCPP_INFO(node->get_logger(), "吸盘法线对齐度(应为0) = %f", normal_alignment);
    RCLCPP_INFO(node->get_logger(), "吸盘方向z分量:%f (应接近零)", suction_dir.z());

    double away_aliignment = suction_dir.dot(away_from_robot);
    RCLCPP_INFO(node->get_logger(), "吸盘远离方向对齐度(应为0) = %f", away_aliignment);

    if(normal_alignment > 0.1 || away_aliignment < 0.9 || std::abs(suction_dir.z()) > 0.1){
        RCLCPP_ERROR(node->get_logger(), "吸盘方向计算错误");
        RCLCPP_WARN(node->get_logger(), "使用备选方案计算吸盘方向");

        Eigen::Vector3d temp = surface_normal.cross(Eigen::Vector3d(0.0, 0.0, 1.0)); // cross 计算叉乘
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

    eef_x_axis = suction_dir;
    RCLCPP_INFO(node->get_logger(), "最终吸盘方向 = (%f, %f, %f)", eef_x_axis.x(), eef_x_axis.y(), eef_x_axis.z());

    Eigen::Vector3d global_x(1.0, 0.0, 0.0);
    Eigen::Vector3d global_y(0.0, 1.0, 0.0);
    Eigen::Vector3d global_z(0.0, 0.0, 1.0);

    if(std::abs(eef_x_axis.dot(global_x)) > 0.9){
        eef_y_axis = global_y.cross(eef_x_axis);
    } else {
        eef_y_axis = global_x.cross(eef_x_axis);
    }

    if(eef_y_axis.norm() < 1e-6){
        eef_y_axis = global_z.cross(eef_x_axis);
    }
    eef_y_axis.normalize();

    RCLCPP_INFO(node->get_logger(), "末端Y轴方向 = (%f, %f, %f)", eef_y_axis.x(), eef_y_axis.y(), eef_y_axis.z());
    
    RCLCPP_INFO(node->get_logger(), "末端Z轴方向 = (%f, %f, %f)", eef_z_axis.x(), eef_z_axis.y(), eef_z_axis.z());
    Eigen::Matrix3d R_eef;
    R_eef.col(0) = eef_x_axis;
    R_eef.col(1) = eef_y_axis;
    R_eef.col(2) = eef_z_axis;
    Eigen::Quaterniond q_eef(R_eef);

    geometry_msgs::msg::Pose result;
    result.position.x = prepare_position.x();
    result.position.y = prepare_position.y();
    result.position.z = prepare_position.z();
    result.orientation.x = q_eef.x();
    result.orientation.y = q_eef.y();
    result.orientation.z = q_eef.z();
    result.orientation.w = q_eef.w();

    grasp_pose.position.x = grasp_position.x();
    grasp_pose.position.y = grasp_position.y();
    grasp_pose.position.z = grasp_position.z(); 
    grasp_pose.orientation.x = q_eef.x();
    grasp_pose.orientation.y = q_eef.y();
    grasp_pose.orientation.z = q_eef.z();
    grasp_pose.orientation.w = q_eef.w();
    
    RCLCPP_INFO(
        node->get_logger(), 
        "======================= 计算结果汇总 ========================"
    );
    RCLCPP_INFO(
        node->get_logger(), 
        "抓取位姿 = Pos(%f, %f, %f)， Ori(%f, %f, %f, %f)", 
        grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z,
        grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w
    );
    RCLCPP_INFO(
        node->get_logger(), 
        "准备位姿 = Pos(%f, %f, %f)， Ori(%f, %f, %f, %f)", 
        result.position.x, result.position.y, result.position.z,
        result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w
    );

    Eigen::Vector3d diff = prapare_position - grasp_position;
    RCLCPP_INFO (node->get_Logger(), "准备位置-抓取位置向量 = (%f, %f, %f)",
        diff.x(), diff.y(), diff.z());

    double robot_aligenment = eef_x_axis.dot(away_from_robot);
    RCLCPP_INFO(node->get_logger(),"吸盘于远离机器人方向对齐度 = %f(应为1)", robot_aligenment);

    return result;
}




geometry_msgs::msg::Pose ArmHandleNode::calculate_prepare_pos_with_orientation(
    const geometry_msgs::msg::Pose& box_pos, 
    double approach_distance, 
    geometry_msgs::msg::Pose &grasp_pose, 
    ApproachMode mode)
{
    Eigen::Vector3d object_center(box_pos.position.x, box_pos.position.y, box_pos.position.z);
    Eigen::Quaterniond q(box_pos.orientation.w, box_pos.orientation.x, 
                         box_pos.orientation.y, box_pos.orientation.z);
    
    Eigen::Vector3d approach_direction(1.0, 0.0, 0.0);
    
    const double object_half_size_x = 0.35;
    Eigen::Vector3d grasp_position = object_center - object_half_size_x * approach_direction;
    Eigen::Vector3d prepare_position = grasp_position + approach_distance * approach_direction;
    
    Eigen::Quaterniond q_eef(1.0, 0.0, 0.0, 0.0);
    
    geometry_msgs::msg::Pose result;
    result.position.x = prepare_position.x();
    result.position.y = prepare_position.y();
    result.position.z = prepare_position.z();
    result.orientation.w = q_eef.w();
    result.orientation.x = q_eef.x();
    result.orientation.y = q_eef.y();
    result.orientation.z = q_eef.z();
    
    grasp_pose.position.x = grasp_position.x();
    grasp_pose.position.y = grasp_position.y();
    grasp_pose.position.z = grasp_position.z();
    grasp_pose.orientation.w = q_eef.w();
    grasp_pose.orientation.x = q_eef.x();
    grasp_pose.orientation.y = q_eef.y();
    grasp_pose.orientation.z = q_eef.z();
    
    return result;
}

bool RobotisTask::add_attached_kfs_collision(){
    moveit_msgs::msg::AttachedCollisionObject collision_object;
    collision_object.link_name = "link6";
    collision_oject.object.header.frame_id = "link6";
    collision_object.object.id = "kfs";
    shape_msgs::msg::SolidPrimitive primitive;   // SolidPrimitive 为一个物体的形状

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.35;
    primitive.dimensions[primitive.BOX_Y] = 0.35;
    primitive.dimensions[primitive.BOX_Z] = 0.35;

    collision_object.object.primitives.push_back(primitive);
    collision_object.object.primitive_poses.push_back(attached_kfs_pos);
    collision_object.object.operation = collision_object.object.ADD;
    psi->applyAttachedCollisionObject(collision_object);
    return true;

}

bool ArmHandleNode::remove_attached_kfs_collision() {
    moveit_msgs::msg::AttachedCollisionObject collision_object;
    collision_object.link_name              = "link6";
    collision_object.object.header.frame_id = "link6";
    collision_object.object.id              = "kfs";
    collision_object.object.operation       = collision_object.object.REMOVE;
    psi->applyAttachedCollisionObject(collision_object);

    remove_kfs_collision("kfs", move_group_interface->getPlanningFrame());
    return true;
}

bool RoboticTask::add_kfs_collision(
    const geometry_msgs::msg::Pose& pose, 
    const std::string& object_id, 
    const std::string& frame_id 
){
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = object_id;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3); 
    primitive.dimensions[primitive.BOX_X] = 0.35;
    primitive.dimensions[primitive.BOX_Y] = 0.35;
    primitive.dimensions[primitive.BOX_Z] = 0.35;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    psi->applyCollisionObject(collision_object);
    return true;
}


bool RoboticTask::remove_kfs_collision(
    const std::string& object_id, 
    const std::string& frame_id
){
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = object_id;
    collision_object.operation = collision_object.REMOVE;
    psi->applyCollisionObject(collision_object);
    return true;
}



bool RoboticTask::set_air_pump(bool enable){
    RCLCPP_INFO(node->get_logger(), "设置气泵参数： %s", enable ? "开启" : "关闭");

    auto node_names = node->get_node_names();
    if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
        RCLCPP_ERROR(node->get_logger(), "没有气泵驱动节点");
        return false;
    }

    auto temp_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/driver_node");

    if(!temp_client->wait_for_service(1s)){
        RCLCPP_ERROR(node->get_logger(), "driver_node 参数不可用 ");
        return false;
    }

    auto future = temp_client->set_parameters({
        rclcpp::Parameter("enable_air_pump", enable)
    });

    try{
        const auto& result = future.get();

        if (result.empty()){
            RCLCPP_ERROR(node->get_logger(), "返回结果为空");
            return false;
        }

        // 检查第一个参数的结果
        const auto& first_result = result.front();
        if(first_result.successful){
            RCLCPP_INFO(node->get_logger(), "设置成功");
            return true;
        } else {
            RCLCPP_ERROR(node->get_logger(), "设置失败： %s", first_result.reason);
            return false;
        }
    } catch (const std::exception& e){
        RCLCPP_ERROR(node->get_logger(), "设置失败： %s", e.what());
        return false;
    }
}

// 获取关节位置
Eigen::VectorXd RoboticTask::get_joint_position() const {
    return joint_position;
}

















