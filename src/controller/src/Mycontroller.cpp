/**
 * @file Mycontroller.cpp
 * @author <kangy>
 * @date 2026-3-04
 * @brief 实现控制器与硬件之间的通信
 */

/**
 * @brief ROS2 硬件接口实现教程
 *
 * 这个文件演示了如何在 ROS2 中实现一个硬件接口 (Hardware Interface)。
 * 硬件接口是 ROS2 控制框架的一部分，用于与实际硬件进行通信。
 *
 * 关键概念：
 * - ROS2 控制框架：分为控制器 (Controller) 和硬件接口 (Hardware Interface)
 *   - 控制器：实现控制逻辑，如 PID 控制、轨迹跟踪等
 *   - 硬件接口：负责与硬件的 I/O 操作，读取传感器数据，发送命令到执行器
 *
 * - 硬件接口类型：
 *   - SystemInterface：系统级硬件接口，管理多个关节
 *   - SensorInterface：传感器接口
 *   - ActuatorInterface：执行器接口
 *
 * - 生命周期：
 *   - on_init：初始化时调用
 *   - on_configure：配置时调用
 *   - on_activate：激活时调用
 *   - on_deactivate：停用时调用
 *   - on_cleanup：清理时调用
 *   - on_shutdown：关闭时调用
 *
 * - 接口：
 *   - StateInterface：状态接口，用于读取硬件状态（如位置、速度）
 *   - CommandInterface：命令接口，用于发送命令到硬件（如位置、速度、力矩）
 *
 * 这个实现还包含了一个 ROS2 节点，用于发布和订阅主题，这在标准硬件接口中不常见，
 * 但可以用于测试或与 MoveIt2 等其他组件通信。
 *
 * 对于 MoveIt2 小白：
 * MoveIt2 是 ROS2 的运动规划框架，用于机器人手臂的路径规划和执行。
 * 硬件接口是 MoveIt2 与实际机器人硬件连接的桥梁。
 *
 * 使用方法：
 * 1. 编译这个包
 * 2. 在 robot_description 中配置硬件接口
 * 3. 启动 controller_manager 和这个硬件接口
 *
 */

// ROS2 硬件接口相关头文件
#include <hardware_interface/system_interface.hpp> // 硬件接口的基类，提供生命周期管理
#include <pluginlib/class_list_macros.hpp> // 用于插件系统的宏定义，让 ROS2 能发现和加载这个硬件接口
#include <hardware_interface/handle.hpp> // 硬件句柄的定义
#include <memory> // 智能指针

// ROS2 节点和通信相关头文件
#include <rclcpp/node.hpp> // ROS2 节点的基类
#include <rclcpp/publisher.hpp> // 发布者，用于发布消息到主题
#include <rclcpp/subscription.hpp> // 订阅者，用于订阅主题消息
#include <rclcpp/logging.hpp> // 日志系统

// 标准库
#include <vector> // 向量容器
#include <string> // 字符串

// 更多硬件接口头文件
#include <hardware_interface/hardware_info.hpp> // 硬件信息结构
// #include <hardware_interface/state_interface.hpp> // 状态接口
// #include <hardware_interface/command_interface.hpp> // 命令接口
#include <hardware_interface/types/hardware_interface_return_values.hpp> // 返回值类型

// 生命周期状态
#include <rclcpp_lifecycle/state.hpp> // 生命周期状态定义

// 自定义消息
#include <robot_interfaces/msg/robot.hpp> // 自定义的机器人消息，包含关节数据

// 自定义命名空间，避免与其他库冲突
namespace mycontroller {

    // 硬件接口类，继承自 SystemInterface
    // 这个类实现了系统级硬件接口，管理多个关节的 I/O 操作
    // 它提供了状态接口（读取硬件状态）和命令接口（发送命令到硬件）
    class MyControl : public hardware_interface::SystemInterface {
    public:
        // 宏定义，用于生成共享指针相关的代码
        RCLCPP_SHARED_PTR_DEFINITIONS(MyControl)
        
        // 初始化方法，在硬件接口被加载时调用一次
        // 参数：info - 包含从 robot_description 中解析的硬件配置信息，如关节名称、类型、参数等
        // 返回：CallbackReturn::SUCCESS 表示成功，FAILURE 表示失败
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo& info
        ) override {
            // 遍历所有关节，初始化状态和命令向量
            // info.joints 包含了 URDF 中定义的所有关节信息
            for(auto &joint : info.joints){
                // 添加关节名称，用于后续接口创建
                joint_names_.push_back(joint.name);
                // 初始化状态值：位置、速度、力矩（从硬件读取）
                // 注意：这里应该从实际硬件读取初始位置，而不是设为0
                state_positions_.push_back(0.0);
                state_velocities_.push_back(0.0);
                state_efforts_.push_back(0.0);
                // 初始化命令值：位置、速度、力矩（发送到硬件）
                // 设置为初始位置，避免关节归零
                command_positions_.push_back(0.0);
                command_velocities_.push_back(0.0);
                command_efforts_.push_back(0.0);
            }

            // 创建 ROS2 节点，用于通信
            // 注意：标准硬件接口通常不创建 ROS2 节点，这里是为了演示如何与 MoveIt2 等通信
            // 在实际应用中，硬件接口应该直接与硬件通信，不依赖 ROS 主题
            node_ = std::make_shared<rclcpp::Node>("controller_node");

            // 创建发布者，发布目标关节数据到 "myjoints_target" 主题
            // QoS 10 表示队列大小，影响消息缓冲
            publisher_ = node_->create_publisher<robot_interfaces::msg::Robot>("myjoints_target", 10);

            // 创建订阅者，订阅状态关节数据从 "myjoints_state" 主题
            // 使用 lambda 回调函数处理接收到的消息
            subscriber_ = node_->create_subscription<robot_interfaces::msg::Robot>(
                "myjoints_state", 10, 
                [this](const robot_interfaces::msg::Robot& msg){
                    // 回调函数：更新状态向量
                    // 这里假设有 6 个关节，实际应使用 joint_names_.size()
                    for(int i = 0 ; i < 6 ; i++){
                        // 更新位置和速度状态
                        state_positions_[i] = static_cast<double>(msg.joints[i].rad);
                        state_velocities_[i] = static_cast<double>(msg.joints[i].omega);
                        state_efforts_[i] = static_cast<double>(msg.joints[i].torque);
                    }
                    // 添加调试日志
                    RCLCPP_INFO(node_->get_logger(), "接收到关节状态: joint1=%.3f, joint2=%.3f, joint3=%.3f", 
                               state_positions_[0], state_positions_[1], state_positions_[2]);
                }
            );
            return hardware_interface::CallbackReturn::SUCCESS;
        }
        
        // 导出状态接口方法
        // 这个方法返回一个状态接口向量，每个接口代表一个可以被控制器读取的状态
        // 状态接口包括关节的位置、速度和力矩等，用于反馈控制
        // 返回：vector of StateInterface，每个接口包含关节名、接口名（如 "position"）、状态值的引用
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
            std::vector<hardware_interface::StateInterface> state_interfaces;
            // 预留空间，提高效率
            state_interfaces.reserve(joint_names_.size() * 3);
            // 为每个关节创建位置、速度和力矩状态接口
            for(size_t i = 0 ; i < joint_names_.size() ; i++){
                // 位置状态接口：控制器可以读取关节当前位置
                state_interfaces.emplace_back(joint_names_[i], "position", &state_positions_[i]);
                // 速度状态接口：控制器可以读取关节当前速度
                state_interfaces.emplace_back(joint_names_[i], "velocity", &state_velocities_[i]);
                // 力矩状态接口：控制器可以读取关节当前力矩（反馈）
                state_interfaces.emplace_back(joint_names_[i], "effort", &state_efforts_[i]);
            }
            return state_interfaces;
        }

        // 导出命令接口方法
        // 这个方法返回一个命令接口向量，每个接口代表一个可以被控制器写入的命令
        // 命令接口包括关节的目标位置、速度、力矩等
        // 返回：vector of CommandInterface，每个接口包含关节名、接口名、命令值的引用
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
            std::vector<hardware_interface::CommandInterface> command_interfaces;
            // 预留空间
            command_interfaces.reserve(joint_names_.size() * 3);
            // 为每个关节创建位置、速度、力矩命令接口
            for(size_t i = 0 ; i < joint_names_.size() ; ++i){
                // 位置命令接口：控制器可以设置关节目标位置
                command_interfaces.emplace_back(joint_names_[i], "position", &command_positions_[i]);
                // 速度命令接口：控制器可以设置关节目标速度
                command_interfaces.emplace_back(joint_names_[i], "velocity", &command_velocities_[i]);
                // 力矩命令接口：控制器可以设置关节目标力矩
                command_interfaces.emplace_back(joint_names_[i], "effort", &command_efforts_[i]);
            }
            return command_interfaces;
        }



        // 生命周期管理方法
        // 以下方法是硬件接口的生命周期回调，由 ROS2 控制管理器调用

        // 配置方法：在硬件接口被配置时调用
        // 参数：previous_state - 上一个生命周期状态
        // 返回：SUCCESS 表示配置成功
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& /*previous_state*/
        ) override {
            return hardware_interface::CallbackReturn::SUCCESS;
        }
        
        // 激活方法：在硬件接口被激活（开始运行）时调用
        // 参数：previous_state - 上一个生命周期状态
        // 返回：SUCCESS 表示激活成功
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& /*previous_state*/
        ) override {
            return hardware_interface::CallbackReturn::SUCCESS;
        }
        
        // 停用方法：在硬件接口被停用时调用
        // 参数：previous_state - 上一个生命周期状态
        // 返回：SUCCESS 表示停用成功
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& /*previous_state*/
        ) override {
            return hardware_interface::CallbackReturn::SUCCESS;
        }
        
        // 清理方法：在硬件接口被清理时调用
        // 参数：previous_state - 上一个生命周期状态
        // 返回：SUCCESS 表示清理成功
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& /*previous_state*/
        ) override {
            return hardware_interface::CallbackReturn::SUCCESS;
        }
        
        // 关闭方法：在硬件接口被关闭时调用
        // 参数：previous_state - 上一个生命周期状态
        // 返回：SUCCESS 表示关闭成功
        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& /*previous_state*/
        ) override {
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        // 读取方法：周期性调用，用于从硬件读取状态
        // 参数：time - 当前时间，period - 时间周期
        // 返回：OK 表示读取成功
        // 在实际硬件接口中，这里应该与硬件通信，更新 state_positions_ 和 state_velocities_
        hardware_interface::return_type read(
            const rclcpp::Time& time,
            const rclcpp::Duration& period
        ) override {
            // 添加调试日志，检查当前状态
            static int log_counter = 0;
            if (++log_counter >= 100) { // 每100次调用打印一次
                RCLCPP_INFO(node_->get_logger(), "读取关节状态: joint1=%.3f, joint2=%.3f, joint3=%.3f", 
                           state_positions_[0], state_positions_[1], state_positions_[2]);
                log_counter = 0;
            }
            return hardware_interface::return_type::OK;
        }

        // 写入方法：周期性调用，用于向硬件发送命令
        // 参数：time - 当前时间，period - 时间周期
        // 返回：OK 表示写入成功
        // 在实际硬件接口中，这里应该与硬件通信，发送 command_positions_ 等到硬件
        hardware_interface::return_type write(
            const rclcpp::Time& time,
            const rclcpp::Duration& period
        ) override {
            // 创建机器人消息
            robot_interfaces::msg::Robot msg;
            // 填充关节命令数据
            for(int i = 0 ; i < 6 ; i++){
                msg.joints[i].rad = static_cast<float>(command_positions_[i]);      // 位置命令
                msg.joints[i].omega = static_cast<float>(command_velocities_[i]);   // 速度命令
                msg.joints[i].torque = static_cast<float>(command_efforts_[i]);     // 力矩命令
            }
            // 添加调试日志
            static int log_counter = 0;
            if (++log_counter >= 100) { // 每100次调用打印一次
                RCLCPP_INFO(node_->get_logger(), "发送关节命令: joint1=%.3f, joint2=%.3f, joint3=%.3f", 
                           command_positions_[0], command_positions_[1], command_positions_[2]);
                log_counter = 0;
            }
            // 发布消息到主题，让其他节点（如驱动节点）接收
            publisher_->publish(msg);
            return hardware_interface::return_type::OK;
        }
        
    private:
        // 私有成员变量
        
        // ROS2 节点指针，用于创建发布者和订阅者
        // 注意：标准硬件接口不应该创建 ROS 节点，这里是为了演示
        std::shared_ptr<rclcpp::Node> node_;
        
        // 关节名称向量，存储从 URDF 解析的所有关节名称
        std::vector<std::string> joint_names_;
        
        // 订阅者，订阅关节状态消息从 "myjoints_state" 主题
        // 用于接收硬件反馈的状态数据
        rclcpp::Subscription<robot_interfaces::msg::Robot>::SharedPtr subscriber_;
        
        // 发布者，发布关节命令消息到 "myjoints_target" 主题
        // 用于发送控制命令到硬件
        rclcpp::Publisher<robot_interfaces::msg::Robot>::SharedPtr publisher_;

        // 状态向量：存储从硬件读取的关节状态
        std::vector<double> state_positions_;   // 关节位置（弧度）
        std::vector<double> state_velocities_;  // 关节速度（弧度/秒）
        std::vector<double> state_efforts_;     // 关节力矩（反馈）
        
        // 命令向量：存储控制器设置的关节命令
        std::vector<double> command_positions_; // 目标位置
        std::vector<double> command_velocities_; // 目标速度
        std::vector<double> command_efforts_;   // 目标力矩
    };
}

// 插件导出
// 这个宏让 ROS2 的 pluginlib 系统能够发现和加载这个硬件接口
// 参数：完整类名（命名空间::类），基类
// 这允许 controller_manager 通过插件系统加载这个硬件接口
PLUGINLIB_EXPORT_CLASS(mycontroller::MyControl, hardware_interface::SystemInterface)