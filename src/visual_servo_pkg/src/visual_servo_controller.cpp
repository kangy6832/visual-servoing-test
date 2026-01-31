/**
 * @file visual_servo_controller.cpp
 * @brief 视觉伺服控制节点
 * 
 * 功能：订阅目标位置，生成机器人控制指令
 * 订阅话题：/visual_servo/target_position
 * 发布话题：/cmd_vel（机器人速度控制）
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <cmath>

/**
 * @brief 视觉伺服控制器类
 * 
 * 实现基于视觉反馈的机器人控制：
 * - 接收目标位置信息
 * - 获取机器人当前位置
 * - 计算控制偏差
 * - 生成速度指令
 */
class VisualServoController : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param options 节点选项
     */
    VisualServoController(const rclcpp::NodeOptions & options)
        : Node("visual_servo_controller", options)
    {
        // 获取参数
        this->declare_parameter<double>("proportional_gain", 0.5);
        this->declare_parameter<double>("max_linear_velocity", 0.3);
        this->declare_parameter<double>("max_angular_velocity", 1.0);
        this->declare_parameter<double>("target_distance", 0.5);
        this->declare_parameter<double>("deadzone", 0.02);
        
        double kp = this->get_parameter("proportional_gain").as_double();
        double max_lin = this->get_parameter("max_linear_velocity").as_double();
        double max_ang = this->get_parameter("max_angular_velocity").as_double();
        double target_dist = this->get_parameter("target_distance").as_double();
        double deadzone = this->get_parameter("deadzone").as_double();
        
        // 初始化TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 创建订阅者
        target_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/visual_servo/target_position",
            rclcpp::SensorDataQoS(),
            std::bind(&VisualServoController::target_callback, this, std::placeholders::_1));
        
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            rclcpp::SensorDataQoS(),
            std::bind(&VisualServoController::odom_callback, this, std::placeholders::_1));
        
        // 创建发布者
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        // 控制参数
        kp_ = kp;
        max_linear_velocity_ = max_lin;
        max_angular_velocity_ = max_ang;
        target_distance_ = target_dist;
        deadzone_ = deadzone;
        
        RCLCPP_INFO(this->get_logger(), "视觉伺服控制器初始化完成");
        RCLCPP_INFO(this->get_logger(), "目标距离: %.2f m", target_distance_);
    }
    
private:
    /**
     * @brief 目标位置回调函数
     * @param msg 目标位置消息
     */
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        latest_target_ = msg;
    }
    
    /**
     * @brief 里程计回调函数
     * @param msg 里程计消息
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = *msg;
        process_servo_control();
    }
    
    /**
     * @brief 处理伺服控制
     * 
     * 核心控制逻辑：
     * 1. 检查目标数据有效性
     * 2. 计算位置偏差
     * 3. 生成速度指令
     */
    void process_servo_control()
    {
        // 检查是否有有效的目标数据
        if (!latest_target_ || !current_pose_) {
            return;
        }
        
        geometry_msgs::msg::Twist cmd_vel;
        
        try {
            // 将目标点转换到里程计坐标系
            geometry_msgs::msg::PointStamped target_in_base;
            tf_buffer_->transform(*latest_target_, target_in_base, "base_link",
                                 tf2::Duration(std::chrono::milliseconds(100)));
            
            // 计算距离偏差
            double dx = target_in_base.point.x;
            double dy = target_in_base.point.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            
            // 计算角度偏差（目标相对于机器人前进方向）
            double angle_error = std::atan2(dy, dx);
            
            RCLCPP_DEBUG(this->get_logger(), "距离偏差: %.3f m, 角度偏差: %.3f rad",
                        distance, angle_error);
            
            // 死区处理
            if (distance < deadzone_) {
                RCLCPP_INFO(this->get_logger(), "目标到达，停止运动");
                velocity_publisher_->publish(cmd_vel);
                return;
            }
            
            // 计算控制量（比例控制）
            double linear_vel = kp_ * (distance - target_distance_);
            double angular_vel = kp_ * angle_error;
            
            // 限制最大速度
            linear_vel = std::clamp(linear_vel, -max_linear_velocity_, max_linear_velocity_);
            angular_vel = std::clamp(angular_vel, -max_angular_velocity_, max_angular_velocity_);
            
            // 设置速度指令
            cmd_vel.linear.x = linear_vel;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = angular_vel;
            
            // 发布速度指令
            velocity_publisher_->publish(cmd_vel);
            
            RCLCPP_DEBUG(this->get_logger(), "控制指令: v=%.3f, ω=%.3f",
                        linear_vel, angular_vel);
            
        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN(this->get_logger(), "坐标变换失败: %s", e.what());
        }
    }
    
    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    geometry_msgs::msg::PointStamped::SharedPtr latest_target_;
    nav_msgs::msg::Odometry::SharedPtr current_pose_;
    
    // 控制参数
    double kp_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double target_distance_;
    double deadzone_;
};

/**
 * @brief 主函数入口
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VisualServoController>(
        rclcpp::NodeOptions().use_intra_process_comms(true));
    
    RCLCPP_INFO(node->get_logger(), "视觉伺服控制器启动");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
