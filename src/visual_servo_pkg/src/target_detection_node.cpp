/**
 * @file target_detection_node.cpp
 * @brief 目标检测节点
 * 
 * 功能：订阅深度图像，检测目标物体并发布位置信息
 * 订阅话题：/camera/depth/image_raw
 * 发布话题：/visual_servo/target_position
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

/**
 * @brief 目标检测器类
 * 
 * 实现基于深度图像的目标检测功能：
 * - 接收深度图像
 * - 检测目标物体位置
 * - 计算三维坐标
 * - 发布检测结果
 */
class TargetDetector : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param options 节点选项
     */
    TargetDetector(const rclcpp::NodeOptions & options)
        : Node("target_detector", options)
    {
        // 获取参数
        this->declare_parameter<double>("depth_threshold_min", 0.1);
        this->declare_parameter<double>("depth_threshold_max", 5.0);
        this->declare_parameter<double>("target_radius", 0.05);
        this->declare_parameter<std::string>("camera_frame", "camera_depth_optical_frame");
        
        double depth_min = this->get_parameter("depth_threshold_min").as_double();
        double depth_max = this->get_parameter("depth_threshold_max").as_double();
        double target_radius = this->get_parameter("target_radius").as_double();
        std::string camera_frame = this->get_parameter("camera_frame").as_string();
        
        // 创建订阅者
        depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&TargetDetector::depth_callback, this, std::placeholders::_1));
        
        // 创建发布者
        target_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/visual_servo/target_position", 10);
        
        target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/visual_servo/target_pose", 10);
        
        // 初始化相机内参（需要根据实际相机标定结果设置）
        fx_ = 525.0;  // 焦距x
        fy_ = 525.0;  // 焦距
        cx_ = 319.5;  // 主点x
        cy_ = 239.5;  // 主点y
        
        RCLCPP_INFO(this->get_logger(), "目标检测节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "深度范围: %.2f - %.2f m", depth_min, depth_max);
    }
    
private:
    /**
     * @brief 深度图像回调函数
     * @param msg 深度图像消息
     */
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 将ROS图像消息转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
            cv::Mat depth_image = cv_ptr->image;
            
            // 检测目标物体
            auto detection_result = detect_target(depth_image);
            
            if (detection_result.found) {
                // 发布检测结果
                publish_detection_result(detection_result, msg->header);
            }
            
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        }
    }
    
    /**
     * @brief 目标检测结构体
     */
    struct DetectionResult {
        bool found;
        int pixel_x;
        int pixel_y;
        float depth;
        float point_x;
        float point_y;
        float point_z;
    };
    
    /**
     * @brief 检测目标物体
     * @param depth_image 深度图像
     * @return 检测结果
     */
    DetectionResult detect_target(const cv::Mat & depth_image)
    {
        DetectionResult result;
        result.found = false;
        
        if (depth_image.empty()) {
            return result;
        }
        
        // 深度阈值分割
        cv::Mat depth_mask;
        cv::inRange(depth_image, cv::Scalar(0.1), cv::Scalar(5.0), depth_mask);
        
        // 形态学处理
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_CLOSE, kernel);
        
        // 找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(depth_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (contours.empty()) {
            return result;
        }
        
        // 找最大轮廓
        size_t max_contour_idx = 0;
        double max_area = 0;
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_contour_idx = i;
            }
        }
        
        // 计算轮廓矩和中心点
        std::vector<cv::Point> largest_contour = contours[max_contour_idx];
        cv::Moments moments = cv::moments(largest_contour);
        
        if (moments.m00 > 0) {
            int center_x = static_cast<int>(moments.m10 / moments.m00);
            int center_y = static_cast<int>(moments.m01 / moments.m00);
            
            // 获取深度值
            float depth_value = depth_image.at<float>(center_y, center_x);
            
            // 检查深度值有效性
            if (depth_value > 0.1 && depth_value < 5.0) {
                // 计算三维坐标
                float world_x = (center_x - cx_) * depth_value / fx_;
                float world_y = (center_y - cy_) * depth_value / fy_;
                float world_z = depth_value;
                
                result.found = true;
                result.pixel_x = center_x;
                result.pixel_y = center_y;
                result.depth = depth_value;
                result.point_x = world_x;
                result.point_y = world_y;
                result.point_z = world_z;
                
                RCLCPP_DEBUG(this->get_logger(), "检测到目标: (%.3f, %.3f, %.3f) m",
                           world_x, world_y, world_z);
            }
        }
        
        return result;
    }
    
    /**
     * @brief 发布检测结果
     * @param result 检测结果
     * @param header 消息头
     */
    void publish_detection_result(const DetectionResult & result, 
                                  const std_msgs::msg::Header & header)
    {
        // 发布点消息
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.stamp = this->now();
        point_msg.header.frame_id = camera_frame_;
        point_msg.point.x = result.point_x;
        point_msg.point.y = result.point_y;
        point_msg.point.z = result.point_z;
        target_position_pub_->publish(point_msg);
        
        // 发布位姿消息（假设目标朝向相机）
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = point_msg.header;
        pose_msg.pose.position = point_msg.point;
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;
        target_pose_pub_->publish(pose_msg);
    }
    
    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    
    // 相机内参
    double fx_, fy_, cx_, cy_;
    std::string camera_frame_;
};

/**
 * @brief 主函数入口
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TargetDetector>(
        rclcpp::NodeOptions().use_intra_process_comms(true));
    
    RCLCPP_INFO(node->get_logger(), "目标检测节点启动");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
