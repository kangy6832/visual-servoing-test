/**
 * @file robotic_task_node.cpp
 * @brief Main executable for robotic task management
 * @date 2026-02-05
 */

#include "control_pack/robotic_task.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create ROS2 node
    auto node = std::make_shared<rclcpp::Node>("robotic_task_node");
    
    RCLCPP_INFO(node->get_logger(), "Starting robotic task node...");
    
    try {
        // Create robotic task instance
        auto robotic_task = std::make_shared<robotic_task::RoboticTask>(node);
        
        RCLCPP_INFO(node->get_logger(), "Robotic task initialized successfully");
        
        // Run ROS2 spin
        rclcpp::spin(node);
        
        RCLCPP_INFO(node->get_logger(), "Shutting down robotic task node...");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize robotic task: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    // Shutdown ROS2
    rclcpp::shutdown();
    
    return 0;
}