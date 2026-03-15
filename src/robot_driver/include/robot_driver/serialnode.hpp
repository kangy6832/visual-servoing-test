/**
 * @file serialnode.hpp
 * @brief 串行通信ROS2节点类
 * 
 * 核心目的：
 * ---------------------
 * 作为机器人驱动系统的核心ROS2节点，负责在ROS2运动规划系统与真实机器人硬件之间建立通信桥梁。
 * 该节点将MoveIt生成的运动指令转换为机器人控制器能理解的协议，同时将机器人状态实时反馈给ROS2系统，
 * 实现基于视觉伺服的闭环控制。
 * 
 * 具体功能：
 * -------------------
 * 1. ROS2话题通信：订阅运动控制指令，发布机器人状态信息
 * 2. USB CDC通信：通过USB虚拟串口与机器人控制器交换数据
 * 3. 多线程管理：创建专用线程处理USB事件和控制指令发送
 * 4. 参数服务器：支持运行时动态配置节点参数
 * 5. 状态监控：实时记录和报告通信状态
 * 
 * 系统架构：
 * ---------
 * - ROS2层：作为标准ROS2节点，集成到MoveIt运动规划框架
 * - 通信层：使用CDCTrans类处理USB通信
 * - 数据层：使用Arm_t结构体传输机器人状态
 * - 控制层：实现运动指令的转换和发送
 * 
 * 通信流程：
 * ---------
 * 1. 订阅"myjoints_target"话题，接收MoveIt生成的运动指令
 * 2. 将ROS2消息转换为Arm_t结构体
 * 3. 通过USB CDC发送到机器人控制器
 * 4. 从控制器接收机器人状态数据
 * 5. 将状态数据发布到"myjoints_state"话题
 * 6. MoveIt和其他节点订阅状态话题实现闭环控制
 * 
 * 使用场景：
 * ---------
 * - 真实机器人的视觉伺服控制
 * - 基于MoveIt的运动规划执行
 * - 机器人状态实时监控
 * - 硬件在环仿真测试
 * 
 * @author 系统开发者
 * @version 1.0
 * @date 2026年
 * @copyright 保留所有权利
 */

#ifndef __SERIALNODE_HPP__
#define __SERIALNODE_HPP__

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <cdc_trans.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <data_pack.h>

/**
 * @brief 串行通信ROS2节点类
 * 
 * @details 该类继承自rclcpp::Node，是机器人驱动系统的核心组件。
 *          负责管理ROS2话题通信、USB设备连接、多线程处理和状态监控。
 * 
 * 设计特点：
 * 1. 继承ROS2节点：集成到ROS2生态系统
 * 2. RAII资源管理：确保资源正确释放
 * 3. 多线程安全：使用原子变量和互斥锁保护共享数据
 * 4. 异常安全：确保异常情况下的系统稳定性
 * 5. 可配置性：支持运行时参数调整
 * 
 * 节点名称：driver_node
 * 发布话题：myjoints_state (robot_interfaces::msg::Arm)
 * 订阅话题：myjoints_target (robot_interfaces::msg::Arm)
 * 
 * @note 需要以足够的权限运行以访问USB设备
 * @warning 确保机器人控制器已正确连接并上电
 * @see CDCTrans USB通信类
 * @see Arm_t 机器人数据结构
 */
class SerialNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * 
     * @details 初始化SerialNode节点，执行以下操作：
     *          1. 初始化ROS2节点，名称为"driver_node"
     *          2. 声明运行时参数（如气泵使能）
     *          3. 创建话题发布器和订阅器
     *          4. 初始化CDCTrans通信对象
     *          5. 注册数据接收回调函数
     *          6. 打开USB设备连接
     *          7. 创建处理线程（USB事件处理和控制指令发送）
     * 
     * @note 构造函数中完成所有初始化工作，确保节点创建后立即可用
     * @warning 如果USB设备打开失败，节点将无法正常工作
     */
    SerialNode();

    /**
     * @brief 析构函数
     * 
     * @details 安全关闭节点，执行以下操作：
     *          1. 设置退出标志，通知线程退出
     *          2. 等待所有线程安全结束
     *          3. 关闭USB设备连接
     *          4. 释放所有分配的资源
     * 
     * @note 确保所有资源被正确释放，避免资源泄漏
     * @warning 析构函数可能阻塞，等待线程退出
     */
    ~SerialNode();

private:
    /**
     * @brief 运动指令订阅回调函数
     * 
     * @param msg 接收到的运动指令消息
     * 
     * @details 当订阅到"myjoints_target"话题的新消息时调用。
     *          将ROS2消息转换为Arm_t结构体，并更新arm_target成员变量。
     *          控制指令由专用线程定期发送到机器人控制器。
     * 
     * 处理流程：
     * 1. 提取6个关节的角度、角速度和力矩信息
     * 2. 更新arm_target结构体
     * 3. 记录订阅计数，定期打印日志
     * 
     * @note 回调函数在ROS2的executor线程中调用
     * @warning 避免在回调函数中进行耗时操作
     */
    void legsSubscribCb(const robot_interfaces::msg::Robot &msg);

    /**
     * @brief 发布机器人状态函数
     * 
     * @param arm_state 从机器人控制器接收到的状态数据指针
     * 
     * @details 将接收到的机器人状态数据转换为ROS2消息并发布到"myjoints_state"话题。
     *          同时处理关节6的特殊情况（欺骗MoveIt认为不存在的关节到达目标位置）。
     * 
     * 处理流程：
     * 1. 验证数据包长度和类型
     * 2. 提取6个关节的状态信息
     * 3. 设置关节6的角度为当前目标值（兼容MoveIt）
     * 4. 发布ROS2消息
     * 5. 记录发布计数，定期打印日志
     * 
     * @note 在CDCTrans的数据接收回调中调用
     * @warning 确保arm_state指针有效且数据格式正确
     */
    void publishLegState(const Arm_t *arm_state);

    bool exit_thread; /**< 线程退出标志，控制工作线程的退出 */

    std::unique_ptr<CDCTrans> cdc_trans; /**< USB CDC通信对象，管理USB设备连接和数据传输 */
    
    /** @brief USB事件处理线程，循环调用CDCTrans::process_once()处理USB事件 */
    std::unique_ptr<std::thread> usb_event_handle_thread;
    
    /** @brief 控制指令发送线程，定期发送arm_target到机器人控制器 */
    std::unique_ptr<std::thread> target_send_thread;
    
    /** @brief 关节状态发布器，将机器人状态发布到"myjoints_state"话题 */
    rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr joint_publisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_moveit;
    
    /** @brief 运动指令订阅器，订阅"myjoints_target"话题接收MoveIt生成的运动指令 */
    rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr joint_subscriber;
    
    /** @brief 参数服务器回调句柄，支持运行时动态修改节点参数 */
    OnSetParametersCallbackHandle::SharedPtr param_server_handle;

    Arm_t arm_target; /**< 当前目标状态，包含6个关节的目标角度、角速度和力矩 */

    std::vector<double> joint_pos; /**< 关节位置缓存（当前未使用） */
    std::vector<double> joint_vel; /**< 关节速度缓存（当前未使用） */

    int subscrib_cnt{20};  /**< 订阅计数阈值，每接收20条消息打印一次日志 */
    int publish_cnt{100};  /**< 发布计数阈值，每发布100条消息打印一次日志 */
    int cur_sub_cnt{0};    /**< 当前订阅计数，用于控制日志输出频率 */
    int cur_pub_cnt{0};    /**< 当前发布计数，用于控制日志输出频率 */
};

#endif // __SERIALNODE_HPP__