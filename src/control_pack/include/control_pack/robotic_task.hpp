#pragma once 

#include "robot_pose_polynomial.hpp"
#include "velocity_ik_generator.hpp"


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
#include <robot_interfaces/msg/arm.hpp> 
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

typedef enum{
    ROBOTIC_ARM_TASK_MOVE = 1,
    ROBOTIC_ARM_TASK_CATCH_TARGET = 2,
    ROBOTIC_ARM_TASK_PLACE_TARGET = 3
}ArmTask;

/**
 * @brief 任务类型
 */
struct TaskType{

}

namespace robotic_task {
    class RoboticTask{
        public:
            explicit RoboticTask(const rclcpp::Node::SharedPtr node);
            ~RoboticTask();

            enum class Exercise {
                POSITION, 
                VELOCITY
            }
        
        private:
            bool success;
            rclcpp::Node::SharedPtr node;
            
            std::mutex task_mutex_;
            std::condition_variable task_cv_;
            bool has_new_task_{false};

            rclcpp::AsyncParametersClient::SharedPtr param_client;
            std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> current_goal_handle;
            moveit::core::RobotModelConstPtr robot_module;
            std::unique_ptr<std::thread> arm_task_thread;
            geometry_msgs::msg::Pose task_target_pos;
            std::atomic<bool> is_running_arm_task{false};





            std::unique_ptr<tf2_ros::Buffer> camera_link0_tf_buffer;;
            std::shared_ptr<tf2_ros::TransformListener> camera_link0_tf_listener_;
            geometry_msgs::msg::TransformStamped camera_link0_tf;


            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;

            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mark_pub_;

            geometry_msgs::msg::Pose attached_kfs_pos;

            std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi;

            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID& uuid, 
                std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal
            )
            rclcpp_action::GoalRespone cancle_goal(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle
            );
            void handle_accepted(
                const std::shared_ptr<rclcpp_action::ServerGoalHandel<robot_interfaces::action:Catch>> goal_handle
            );

            void arm_catch_task_handle();
            

    }

}  // namespace robotic_task