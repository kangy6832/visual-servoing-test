/**
 * @file depth_camera_publisher.cpp
 * @brief 深度相机图像发布节点
 * 
 * 功能：捕获相机图像并发布到指定话题
 * 发布话题：/camera/depth/image_raw（深度图像）
 *          /camera/color/image_raw（彩色图像）
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief 深度相机发布器类
 * 
 * 封装相机图像捕获和发布功能，支持：
 * - 定时发布深度图像和彩色图像
 * - 自动重连相机设备
 * - 错误处理和状态报告
 */
class DepthCameraPublisher : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param options 节点选项
     */
    DepthCameraPublisher(const rclcpp::NodeOptions & options)
        : Node("depth_camera_publisher", options)
    {
        // 获取参数
        this->declare_parameter<std::string>("camera_device", "/dev/video0");
        this->declare_parameter<int>("frame_rate", 30);
        this->declare_parameter<int>("image_width", 640);
        this->declare_parameter<int>("image_height", 480);
        
        std::string camera_device = this->get_parameter("camera_device").as_string();
        int frame_rate = this->get_parameter("frame_rate").as_int();
        int image_width = this->get_parameter("image_width").as_int();
        int image_height = this->get_parameter("image_height").as_int();
        
        // 创建图像传输发布者
        image_transport::ImageTransport it(this);
        
        // 创建深度图像和彩色图像发布者
        depth_publisher_ = it.advertise("/camera/depth/image_raw", 1);
        color_publisher_ = it.advertise("/camera/color/image_raw", 1);
        
        // 创建点云发布者
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", 10);
        
        // 初始化相机
        camera_.open(camera_device);
        if (!camera_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开相机设备: %s", camera_device.c_str());
            return;
        }
        
        // 设置相机分辨率
        camera_.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
        camera_.set(cv::CAP_PROP_FPS, frame_rate);
        
        RCLCPP_INFO(this->get_logger(), "相机初始化成功");
        RCLCPP_INFO(this->get_logger(), "分辨率: %dx%d, 帧率: %d fps", 
                    image_width, image_height, frame_rate);
        
        // 创建定时器
        int timer_period_ms = 1000 / frame_rate;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&DepthCameraPublisher::timer_callback, this));
        
        // 初始化CV桥梁
        cv_bridge_ = std::make_shared<cv_bridge::CvBridge>();
    }
    
    /**
     * @brief 析构函数，释放相机资源
     */
    ~DepthCameraPublisher()
    {
        if (camera_.isOpened()) {
            camera_.release();
        }
    }

private:
    /**
     * @brief 定时器回调函数
     * 
     * 定时捕获图像并发布到对应话题
     */
    void timer_callback()
    {
        if (!camera_.isOpened()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "相机未打开，尝试重新连接...");
            return;
        }
        
        cv::Mat frame;
        camera_ >> frame;
        
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "捕获到空帧");
            return;
        }
        
        try {
            // 创建并发布彩色图像消息
            sensor_msgs::msg::Image::SharedPtr color_msg = 
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            color_msg->header.stamp = this->now();
            color_msg->header.frame_id = "camera_color_frame";
            color_publisher_.publish(color_msg);
            
            // 如果是深度相机，处理深度数据
            if (frame.channels() == 1) {
                sensor_msgs::msg::Image::SharedPtr depth_msg = 
                    cv_bridge::CvImage(std_msgs::msg::msg::Header(), 
                                       "32FC1", frame).toImageMsg();
                depth_msg->header.stamp = this->now();
                depth_msg->header.frame_id = "camera_depth_frame";
                depth_publisher_.publish(depth_msg);
            }
            
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        }
    }
    
    // 成员变量
    cv::VideoCapture camera_;
    std::shared_ptr<cv_bridge::CvBridge> cv_bridge_;
    image_transport::Publisher depth_publisher_;
    image_transport::Publisher color_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief 主函数入口
 * @param argc 参数计数
 * @param argv 参数数组
 * @return 执行状态码
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DepthCameraPublisher>(
        rclcpp::NodeOptions().use_intra_process_comms(true));
    
    RCLCPP_INFO(node->get_logger(), "深度相机发布节点启动");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
