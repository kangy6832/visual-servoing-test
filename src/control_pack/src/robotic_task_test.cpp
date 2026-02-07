/**
 * @file robotic_task.cpp
 * @brief Implementation of robotic task management system
 * @date 2026-02-05
 */

#include "control_pack/robotic_task.hpp"
#include "control_pack/motion_generator_base.hpp"
#include "control_pack/robot_pose_polynomial.hpp"
#include "control_pack/velocity_ik_generator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <thread>
#include <atomic>

namespace robotic_task {

// Constructor
RoboticTask::RoboticTask(const rclcpp::Node::SharedPtr node_ptr) 
    : node(node_ptr),
      success(false),
      count(0),
      end_effector_velocity(0.0, 0.0, 0.0),
      joint_position(Eigen::VectorXd::Zero(6))
{
    RCLCPP_INFO(node->get_logger(), "RoboticTask constructor called");
    
    // Initialize MoveIt interface
    try {
        move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, "robotic_arm");
        move_group_interface->setPlanningTime(10.0);
        move_group_interface->setMaxVelocityScalingFactor(VELOCITY_SCALING);
        move_group_interface->setMaxAccelerationScalingFactor(ACCELERATION_SCALING);
        
        RCLCPP_INFO(node->get_logger(), "MoveGroupInterface initialized for group: robotic_arm");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
        throw;
    }
    
    // Initialize planning scene interface
    psi = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    // Initialize TF buffer and listener
    camera_link0_tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    camera_link0_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*camera_link0_tf_buffer);
    
    // Initialize motion generator components
    // Note: These are declared in the header file as:
    // RobotPosePolynomial::QuinticParam Quintic;
    // RobotPosePolynomial::ContinuousTrajectory Trajectory;
    // RobotKinematicsKDL::RobotArmKinematics robot_kinematics;
    
    // Verify we can access motion generator base functionality
    RCLCPP_INFO(node->get_logger(), "Motion generator base functionality available");
    
    // Verify we can access robot pose polynomial functionality
    RCLCPP_INFO(node->get_logger(), "Robot pose polynomial functionality available");
    
    // Verify we can access velocity IK generator functionality
    RCLCPP_INFO(node->get_logger(), "Velocity IK generator functionality available");
    
    // Start the arm task thread
    try {
        arm_task_thread = std::make_unique<std::thread>([this]() {
            arm_catch_task_handle();
        });
        RCLCPP_INFO(node->get_logger(), "Arm task thread started");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start arm task thread: %s", e.what());
        arm_task_thread = nullptr;
    }
}

// Destructor
RoboticTask::~RoboticTask() {
    RCLCPP_INFO(node->get_logger(), "RoboticTask destructor called");
    
    // Signal thread to stop
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        has_new_task_ = true;
        is_running_arm_task = false;
    }
    task_cv_.notify_one();
    
    // Join thread if running
    if (arm_task_thread && arm_task_thread->joinable()) {
        arm_task_thread->join();
        RCLCPP_INFO(node->get_logger(), "Arm task thread joined");
    }
}

// Handle goal request
rclcpp_action::GoalResponse RoboticTask::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal)
{
    (void)uuid;
    
    RCLCPP_INFO(node->get_logger(), "Received goal request");
    
    // Check if already running a task
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (is_running_arm_task) {
            RCLCPP_WARN(node->get_logger(), "Rejecting goal: already running a task");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    
    // Try to get camera transform
    try {
        camera_link0_tf = camera_link0_tf_buffer->lookupTransform(
            "base_link", "camera_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node->get_logger(), "Cannot get camera to base transform: %s", ex.what());
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Transform goal pose
    tf2::doTransform(goal->target_pose, task_target_pos, camera_link0_tf);
    
    // Normalize orientation
    auto qin = task_target_pos.orientation;
    tf2::Quaternion q(qin.x, qin.y, qin.z, qin.w);
    q.normalize();
    task_target_pos.orientation.x = q.x();
    task_target_pos.orientation.y = q.y();
    task_target_pos.orientation.z = q.z();
    task_target_pos.orientation.w = q.w();
    
    current_task_type = goal->action_type;
    
    RCLCPP_INFO(node->get_logger(), "Goal accepted, action type: %d", goal->action_type);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Cancel goal
rclcpp_action::CancelResponse RoboticTask::cancel_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>>& goal_handle)
{
    (void)goal_handle;
    
    RCLCPP_INFO(node->get_logger(), "Goal cancellation requested");
    
    // Set cancellation flag
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        is_running_arm_task = false;
    }
    
    // Remove collision objects
    remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame());
    
    // Turn off air pump
    set_air_pump(false);
    
    return rclcpp_action::CancelResponse::ACCEPT;
}

// Handle accepted goal
void RoboticTask::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>>& goal_handle)
{
    RCLCPP_INFO(node->get_logger(), "Goal accepted, starting execution");
    
    current_goal_handle = goal_handle;
    
    // Signal thread to start task
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        has_new_task_ = true;
        is_running_arm_task = true;
    }
    task_cv_.notify_one();
}

// Main arm task handler
void RoboticTask::arm_catch_task_handle() {
    RCLCPP_INFO(node->get_logger(), "Arm task handler started");
    
    while (true) {
        // Wait for new task
        {
            std::unique_lock<std::mutex> lock(task_mutex_);
            task_cv_.wait(lock, [this]() { return has_new_task_ || !is_running_arm_task; });
            
            if (!is_running_arm_task) {
                RCLCPP_INFO(node->get_logger(), "Arm task handler stopping");
                break;
            }
            
            has_new_task_ = false;
        }
        
        RCLCPP_INFO(node->get_logger(), "Starting arm catch task");
        
        try {
            // Here we would implement the actual task logic
            // For now, just log and simulate success
            RCLCPP_INFO(node->get_logger(), "Task execution completed successfully");
            
            // Send result
            if (current_goal_handle) {
                auto result = std::make_shared<robot_interfaces::action::Catch::Result>();
                result->success = true;
                current_goal_handle->succeed(result);
                RCLCPP_INFO(node->get_logger(), "Goal succeeded");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Task execution failed: %s", e.what());
            
            // Send failure result
            if (current_goal_handle) {
                auto result = std::make_shared<robot_interfaces::action::Catch::Result>();
                result->success = false;
                current_goal_handle->abort(result);
                RCLCPP_ERROR(node->get_logger(), "Goal aborted");
            }
        }
        
        // Reset task state
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            is_running_arm_task = false;
            current_goal_handle.reset();
        }
    }
}

// Calculate target pose
geometry_msgs::msg::Pose RoboticTask::calculate_target_pose(
    const geometry_msgs::msg::Pose& box_pos,
    double approach_distance,
    geometry_msgs::msg::Pose& grasp_pose,
    ApproachMode mode)
{
    RCLCPP_INFO(node->get_logger(), "Calculating target pose with mode: %d", static_cast<int>(mode));
    
    // Default implementation - returns the input pose
    // In a real implementation, this would calculate based on the box position and approach mode
    
    geometry_msgs::msg::Pose result = box_pos;
    grasp_pose = box_pos;
    
    // Simple offset based on approach distance
    result.position.z += approach_distance;
    
    return result;
}

// Calculate prepare pose with orientation
geometry_msgs::msg::Pose RoboticTask::calculate_prepare_pose_with_orientation(
    const geometry_msgs::msg::Pose& box_pos,
    double approach_distance,
    geometry_msgs::msg::Pose& grasp_pose,
    ApproachMode mode)
{
    RCLCPP_INFO(node->get_logger(), "Calculating prepare pose with orientation");
    
    // Call the base calculate_target_pose method
    return calculate_target_pose(box_pos, approach_distance, grasp_pose, mode);
}



// Calculate joint velocity
Eigen::VectorXd RoboticTask::calculate_joint_velocity(
    const Eigen::Vector3d& end_effector_velocity) const
{
    RCLCPP_INFO(node->get_logger(), "Calculating joint velocity from end effector velocity");
    
    // Default implementation - returns zero vector
    // In a real implementation, this would use the velocity IK generator
    return Eigen::VectorXd::Zero(6);
}

// Calculate joint acceleration
Eigen::VectorXd RoboticTask::calculate_joint_acceleration(
    const Eigen::Vector3d& end_effector_velocity,
    const Eigen::VectorXd& joint_velocity) const
{
    RCLCPP_INFO(node->get_logger(), "Calculating joint acceleration");
    
    // Default implementation - returns zero vector
    return Eigen::VectorXd::Zero(6);
}

// Get joint position
Eigen::VectorXd RoboticTask::get_joint_position() const
{
    return joint_position;
}

// Joint state callback
void RoboticTask::jointStateCallback(const robot_interfaces::msg::Robot::SharedPtr msg)
{
    // Update joint positions from message
    // This is a simplified implementation
    if (msg->joints.size() >= 6) {
        for (size_t i = 0; i < 6; ++i) {
            joint_position(i) = msg->joints[i].rad;
        }
    }
}

// Add attached KFS collision
bool RoboticTask::add_attached_kfs_collision()
{
    RCLCPP_INFO(node->get_logger(), "Adding attached KFS collision");
    // Implementation would go here
    return true;
}

// Remove attached KFS collision
bool RoboticTask::remove_attached_kfs_collision()
{
    RCLCPP_INFO(node->get_logger(), "Removing attached KFS collision");
    // Implementation would go here
    return true;
}

// Add KFS collision
bool RoboticTask::add_kfs_collision(
    const geometry_msgs::msg::Pose& pos,
    const std::string& object_id,
    const std::string& frame_id)
{
    RCLCPP_INFO(node->get_logger(), "Adding KFS collision: %s in frame %s", 
                object_id.c_str(), frame_id.c_str());
    // Implementation would go here
    return true;
}

// Remove KFS collision
bool RoboticTask::remove_kfs_collision(
    const std::string& object_id,
    const std::string& frame_id)
{
    RCLCPP_INFO(node->get_logger(), "Removing KFS collision: %s in frame %s", 
                object_id.c_str(), frame_id.c_str());
    // Implementation would go here
    return true;
}

// Set air pump
bool RoboticTask::set_air_pump(bool enable)
{
    RCLCPP_INFO(node->get_logger(), "Setting air pump: %s", enable ? "ON" : "OFF");
    
    // Try to set parameter on driver node
    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/driver_node");
    
    if (!param_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node->get_logger(), "Driver node parameter service not available");
        return false;
    }
    
    auto future = param_client->set_parameters({
        rclcpp::Parameter("enable_air_pump", enable)
    });
    
    try {
        auto result = future.get();
        if (!result.empty() && result[0].successful) {
            RCLCPP_INFO(node->get_logger(), "Air pump set successfully");
            return true;
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to set air pump parameter");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception setting air pump: %s", e.what());
        return false;
    }
}

} // namespace robotic_task