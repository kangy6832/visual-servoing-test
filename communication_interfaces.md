# 机械臂通信接口文档

## 概述

本文档描述了机械臂上位机与下位机之间的通信接口实现，包括ROS2话题接口、USB CDC通信接口以及相关的数据结构。

## 1. ROS2话题接口

### 1.1 运动指令订阅接口

**话题名称**: `myjoints_target`  
**消息类型**: `robot_interfaces::msg::Robot`  
**通信方向**: 上位机 → 下位机  
**功能**: 接收MoveIt生成的运动控制指令

#### 消息结构
```cpp
// robot_interfaces::msg::Robot
struct Robot {
    Joint joints[6];  // 6个关节信息
};

struct Joint {
    double rad;      // 关节角度 (弧度)
    double omega;    // 关节角速度 (rad/s)
    double torque;   // 关节力矩 (N·m)
};
```

#### 回调函数
```cpp
void SerialNode::legsSubscribCb(const robot_interfaces::msg::Robot& msg);
```

### 1.2 机器人状态发布接口

**话题名称**: `myjoints_state`  
**消息类型**: `robot_interfaces::msg::Robot`  
**通信方向**: 下位机 → 上位机  
**功能**: 发布机器人当前状态信息

#### 发布函数
```cpp
void SerialNode::publishLegState(const Arm_t* arm_state);
```

#### 特殊处理
- 关节6的角度设置为当前目标值，用于欺骗MoveIt认为不存在的关节6到达目标位置

## 2. USB CDC通信接口

### 2.1 设备连接管理

#### 设备识别
- **VID**: 0x0483 (STMicroelectronics)
- **PID**: 0x5740
- **接口编号**: 1

#### 连接函数
```cpp
bool CDCTrans::open(uint16_t vid, uint16_t pid);
void CDCTrans::close();
```

### 2.2 数据传输接口

#### USB端点配置
- **EP_OUT**: 0x01 (发送数据到设备)
- **EP_IN**: 0x81 (从设备接收数据)

#### 同步发送接口
```cpp
int CDCTrans::send(const uint8_t* data, int size, unsigned int time_out = 5);
```
- **参数**: 数据指针、数据大小、超时时间(毫秒)
- **返回值**: 
  - >0: 成功发送的字节数
  - -1: 发送失败
  - -2: 设备未连接

#### 结构体发送接口
```cpp
template <typename T>
bool CDCTrans::send_struct(const T& pack, unsigned int time_out = 5);
```
- **要求**: 结构体必须是标准布局和可平凡复制的
- **典型使用**: 发送`Arm_t`结构体

#### 数据接收接口
```cpp
void CDCTrans::regeiser_recv_cb(std::function<void(const uint8_t* data, int size)> recv_cb);
```
- **功能**: 注册数据接收回调函数
- **调用时机**: 当从USB设备接收到数据时

### 2.3 事件处理接口

```cpp
void CDCTrans::process_once();
```
- **功能**: 处理USB事件循环
- **超时**: 50ms
- **调用频率**: 由专用线程循环调用

## 3. 数据结构定义

### 3.1 Arm_t结构体 (下位机通信格式)

```cpp
struct Arm_t {
    int pack_type;        // 数据包类型 (通常为1)
    Joint joints[6];      // 6个关节信息
};

struct Joint {
    double rad;          // 关节角度
    double omega;        // 关节角速度
    double torque;       // 关节力矩
};
```

### 3.2 ROS2消息格式

```cpp
// robot_interfaces::msg::Robot
struct Robot {
    Joint joints[6];
};

struct Joint {
    double rad;
    double omega;
    double torque;
};
```

## 4. 线程管理接口

### 4.1 USB事件处理线程
```cpp
// 在SerialNode构造函数中创建
usb_event_handle_thread = std::make_unique<std::thread>([this]() {
    do {
        cdc_trans->process_once();
    } while (!exit_thread);
});
```

### 4.2 控制指令发送线程
```cpp
// 在SerialNode构造函数中创建
target_send_thread = std::make_unique<std::thread>([this]() {
    do {
        auto now = std::chrono::system_clock::now();
        cdc_trans->send_struct(arm_target);
        std::this_thread::sleep_until(now + 10ms);
    } while (!exit_thread);
});
```

## 5. 通信参数配置

### 5.1 发送频率
- **控制指令**: 每10ms发送一次
- **状态反馈**: 实时接收，无固定频率

### 5.2 缓冲区大小
- **接收缓冲区**: 2048字节
- **数据包大小**: sizeof(Arm_t)

### 5.3 日志控制
- **订阅日志**: 每20条消息打印一次
- **发布日志**: 每100条消息打印一次

## 6. 错误处理机制

### 6.1 USB设备错误
- **设备断开检测**: 自动检测并设置断开标志
- **热插拔支持**: 支持设备断开重连
- **重连机制**: 500ms延时后自动重连

### 6.2 数据验证
- **包长度验证**: 检查接收数据长度是否为sizeof(Arm_t)
- **包类型验证**: 检查pack_type是否为1

## 7. 使用示例

### 7.1 初始化通信
```cpp
// 创建SerialNode节点
SerialNode node;

// 节点自动初始化USB连接和ROS2话题
```

### 7.2 发送运动指令
```cpp
// 通过ROS2话题发送
robot_interfaces::msg::Robot target_msg;
// 填充目标位置...
publisher->publish(target_msg);
```

### 7.3 接收状态反馈
```cpp
// 通过ROS2话题订阅
auto subscriber = node->create_subscription<robot_interfaces::msg::Robot>(
    "myjoints_state", 10, callback_function);
```

## 8. 相关文件位置

- **ROS2节点实现**: `/src/robot_driver/src/serialnode.cpp`
- **USB通信实现**: `/src/robot_driver/src/cdc_trans.cpp`
- **头文件定义**: `/src/robot_driver/include/robot_driver/`
- **消息定义**: `/src/robot_interfaces/`

## 9. 依赖库

- **libusb-1.0**: USB设备访问
- **rclcpp**: ROS2客户端库
- **robot_interfaces**: 自定义消息类型

## 10. 下位机联调注意事项

### 10.1 硬件连接检查

#### USB设备连接
- **设备识别**: 确保下位机USB设备VID为0x0483，PID为0x5740
- **权限设置**: 需要足够的USB设备访问权限，可能需要sudo或配置udev规则
- **接口编号**: 确认下位机使用USB接口1（interface 1）
- **驱动状态**: 检查是否需要分离内核驱动（libusb_detach_kernel_driver）

#### 物理连接
- **USB线缆**: 使用质量良好的USB线，避免接触不良
- **供电稳定**: 确保下位机供电稳定，避免通信中断
- **接地良好**: 防止静电干扰

### 10.2 通信协议对齐

#### 数据包格式
- **包类型**: 下位机发送的数据包pack_type必须为1
- **数据长度**: 发送的数据包长度必须等于sizeof(Arm_t)
- **字节序**: 确保上下位机使用相同的字节序（通常为小端序）
- **数据类型**: double类型在上下位机中占用相同字节数（8字节）

#### 通信频率
- **发送频率**: 上位机每10ms发送一次控制指令
- **接收处理**: 下位机需要能够及时处理接收到的控制指令
- **状态反馈**: 下位机应及时反馈当前状态，避免控制延迟

### 10.3 调试工具和方法

#### 日志监控
```bash
# 查看ROS2节点日志
ros2 topic echo /myjoints_target
ros2 topic echo /myjoints_state

# 查看节点状态
ros2 node info /driver_node
```

#### USB调试
```bash
# 查看USB设备
lsusb
# 查找VID:PID为0483:5740的设备

# 检查USB权限
ls -l /dev/bus/usb/*/*
```

#### 数据验证
- **角度范围**: 检查关节角度是否在合理范围内
- **速度限制**: 验证角速度是否超过机械限制
- **力矩检查**: 监控力矩数据是否异常

### 10.4 常见问题排查

#### 连接问题
- **设备未找到**: 检查USB连接和设备权限
- **接口占用**: 确认USB接口1未被其他程序占用
- **驱动冲突**: 检查是否有内核驱动冲突

#### 通信异常
- **数据丢失**: 检查USB线缆质量和电磁干扰
- **延迟过大**: 优化下位机处理逻辑，提高响应速度
- **包格式错误**: 验证数据包结构和字节对齐

#### 控制问题
- **运动不平滑**: 检查控制指令发送频率和下位机执行周期
- **位置偏差**: 校准关节零点和角度转换系数
- **超限保护**: 确保下位机有完善的限位保护机制

### 10.5 安全注意事项

#### 紧急停止
- **硬件急停**: 确保物理急停按钮可用
- **软件急停**: 实现ROS2急停服务调用
- **通信断开**: 通信中断时下位机应自动停止运动

#### 运动范围
- **软限位**: 在软件中设置关节运动范围限制
- **硬限位**: 确认机械限位开关工作正常
- **碰撞检测**: 实现基本的碰撞检测逻辑

#### 参数调试
- **渐进调试**: 从小幅度运动开始，逐步增加范围
- **速度控制**: 初始调试时使用较低速度
- **力矩限制**: 设置合理的力矩上限

### 10.6 性能优化

#### 通信优化
- **缓冲区管理**: 合理设置发送和接收缓冲区大小
- **错误处理**: 实现快速的重连机制
- **数据压缩**: 考虑数据压缩以减少通信负载

#### 实时性保证
- **线程优先级**: 适当提高通信线程优先级
- **CPU亲和性**: 将通信线程绑定到特定CPU核心
- **中断处理**: 优化USB中断处理逻辑

---

*文档版本: 1.1*  
*更新日期: 2026年*
