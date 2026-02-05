/**
 * @file main.cpp
 * @brief 机器人驱动节点主程序入口
 * 
 * 核心目的（想干什么）：
 * ---------------------
 * 作为机器人驱动节点的程序入口，负责初始化ROS2系统并启动串行通信节点。
 * 该节点是连接ROS2运动规划系统与真实机器人硬件的桥梁，将MoveIt生成的运动指令
 * 转换为机器人控制器能理解的通信协议，同时将机器人状态反馈给ROS2系统。
 * 
 * 具体功能（干了什么）：
 * -------------------
 * 1. 初始化ROS2上下文和运行时环境
 * 2. 创建并启动SerialNode节点实例
 * 3. 进入ROS2事件循环，处理消息和回调
 * 4. 在程序退出时优雅地关闭ROS2系统
 * 
 * 系统角色：
 * --------
 * - ROS2节点：作为独立的ROS2节点运行
 * - 硬件接口：连接ROS2软件层与机器人硬件层
 * - 协议转换：将ROS2消息转换为串行通信协议
 * 
 * 依赖关系：
 * ---------
 * - ROS2 Humble 或更高版本
 * - rclcpp ROS2 C++客户端库
 * - SerialNode 类（串行通信节点）
 * 
 * @author 系统开发者
 * @version 1.0
 * @date 2024年
 * @copyright 保留所有权利
 */

#include <serialnode.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 机器人驱动节点主函数
 * 
 * @details 该函数是机器人驱动节点的程序入口点，负责：
 *          1. 初始化ROS2系统
 *          2. 创建串行通信节点
 *          3. 启动ROS2事件循环
 *          4. 在程序退出时清理资源
 * 
 * 执行流程：
 * 1. 调用rclcpp::init()初始化ROS2系统
 * 2. 创建SerialNode节点的共享指针实例
 * 3. 调用rclcpp::spin()进入事件循环，处理ROS2消息
 * 4. 程序终止时调用rclcpp::shutdown()清理ROS2资源
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出码，0表示成功，非0表示错误
 * 
 * @note 该节点需要以足够的权限运行，以便访问USB设备
 * @warning 如果USB设备未连接或无法打开，节点将无法正常工作
 * @see SerialNode 类实现具体的串行通信功能
 */
int main(int argc, char **argv)
{
    // 初始化ROS2系统，建立与ROS2 master的连接
    rclcpp::init(argc, argv);
    
    // 创建SerialNode节点实例并进入事件循环
    // SerialNode负责与机器人控制器进行串行通信
    rclcpp::spin(std::make_shared<SerialNode>());
    
    // 程序退出时清理ROS2资源
    rclcpp::shutdown();
    
    // 返回成功退出码
    return 0;
}