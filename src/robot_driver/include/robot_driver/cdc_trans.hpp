/**
 * @file cdc_trans.hpp
 * @brief USB CDC（虚拟串口）通信类
 * 
 * 核心目的（想干什么）：
 * ---------------------
 * 提供通过USB CDC（Communication Device Class）接口与机器人控制器通信的功能。
 * 该类封装了libusb库的复杂操作，为上层的机器人驱动节点提供简单易用的USB通信接口，
 * 实现ROS2系统与嵌入式机器人控制器之间的可靠数据交换。
 * 
 * 具体功能（干了什么）：
 * -------------------
 * 1. 管理USB设备的打开、关闭和重连
 * 2. 实现异步数据发送和接收
 * 3. 支持热插拔检测和自动重连
 * 4. 提供结构体序列化发送接口
 * 5. 处理USB事件循环和错误恢复
 * 
 * 技术特点：
 * ---------
 * - 基于libusb-1.0库实现跨平台USB访问
 * - 使用异步传输提高通信效率
 * - 支持热插拔，增强系统鲁棒性
 * - 线程安全设计，支持多线程环境
 * - 提供模板化的结构体发送接口
 * 
 * 通信协议：
 * ---------
 * - 使用批量传输（Bulk Transfer）模式
 * - 端点配置：EP_OUT(0x01)用于发送，EP_IN(0x81)用于接收
 * - 支持标准布局和可平凡复制的结构体传输
 * 
 * 使用场景：
 * ---------
 * - ROS2机器人驱动节点与STM32等嵌入式控制器的通信
 * - 实时机器人状态监控和控制指令传输
 * - 需要可靠USB通信的嵌入式系统调试
 * 
 * @author 系统开发者
 * @version 1.0
 * @date 2026年
 * @copyright 保留所有权利
 */

#ifndef __CDC_TRANS_H__
#define __CDC_TRANS_H__

#include <atomic>
#include <functional>
#include <libusb-1.0/libusb.h>

/**
 * @brief USB CDC通信类
 * 
 * @details 该类封装了通过USB CDC接口与下位机通信的所有功能，包括设备管理、
 *          数据收发、热插拔处理和错误恢复。使用libusb库实现跨平台兼容性。
 * 
 * 设计原则：
 * 1. 资源管理：RAII原则管理USB资源
 * 2. 异常安全：确保异常情况下的资源释放
 * 3. 线程安全：使用原子变量保护共享状态
 * 4. 可扩展性：模板化接口支持多种数据结构
 * 
 * @note 需要以足够的权限运行（如root或配置了USB访问权限的用户）
 * @warning 不支持同时多个实例访问同一USB设备
 * @see libusb-1.0 USB访问库
 */
class CDCTrans {
public:
    static constexpr uint8_t EP_OUT = 0x01; /**< 输出端点地址，用于发送数据到设备 */
    static constexpr uint8_t EP_IN  = 0x81; /**< 输入端点地址，用于从设备接收数据 */

    /**
     * @brief 默认构造函数
     * 
     * @details 初始化CDCTrans对象，设置初始状态和libusb上下文。
     *          不打开任何设备，需要调用open()方法连接具体设备。
     * 
     * @note 构造函数中初始化libusb库上下文
     * @warning 构造函数可能抛出异常，如果libusb初始化失败
     */
    CDCTrans();

    /**
     * @brief 析构函数
     * 
     * @details 清理所有资源，包括：
     *          1. 停止事件处理线程
     *          2. 关闭USB设备连接
     *          3. 释放libusb资源
     * 
     * @note 确保所有资源被正确释放，避免资源泄漏
     */
    ~CDCTrans();

    /**
     * @brief 打开指定VID/PID的USB设备
     * 
     * @param vid 供应商ID（Vendor ID）
     * @param pid 产品ID（Product ID）
     * @return true 设备打开成功
     * @return false 设备打开失败
     * 
     * @details 根据VID/PID查找并打开USB设备，执行以下操作：
     *          1. 查找匹配的USB设备
     *          2. 打开设备句柄
     *          3. 分离内核驱动（如果需要）
     *          4. 申请接口
     *          5. 设置异步传输
     *          6. 注册热插拔回调
     * 
     * @note 默认VID/PID为0x0483/0x5740（STMicroelectronics）
     * @warning 需要足够的权限访问USB设备
     */
    bool open(uint16_t vid, uint16_t pid);

    /**
     * @brief 关闭USB设备连接
     * 
     * @details 安全关闭USB连接，包括：
     *          1. 取消所有未完成的传输
     *          2. 释放接口
     *          3. 关闭设备句柄
     *          4. 清理传输结构体
     * 
     * @note 可以在任何时候调用，即使设备未打开
     */
    void close();

    /**
     * @brief 发送原始数据到USB设备
     * 
     * @param data 要发送的数据指针
     * @param size 数据大小（字节）
     * @param time_out 超时时间（毫秒），默认5ms
     * @return int 实际发送的字节数，负数表示错误
     * 
     * @details 使用同步批量传输发送数据，适用于小数据量传输。
     *          对于大数据量或需要更高性能的场景，建议使用异步传输。
     * 
     * @retval >0 成功发送的字节数
     * @retval -1 发送失败
     * @retval -2 设备未连接
     * 
     * @note 线程安全，但性能较低
     */
    int send(const uint8_t* data, int size, unsigned int time_out=5);

    /**
     * @brief 注册数据接收回调函数
     * 
     * @param recv_cb 接收回调函数，参数为数据指针和大小
     * 
     * @details 设置当从USB设备接收到数据时调用的回调函数。
     *          回调函数在libusb的事件处理线程中调用，需要注意线程安全。
     * 
     * @note 回调函数应尽快返回，避免阻塞事件处理
     * @warning 回调函数中不要进行耗时操作
     */
    void regeiser_recv_cb(std::function<void(const uint8_t* data, int size)> recv_cb);

    /**
     * @brief 发送结构体数据
     * 
     * @tparam T 结构体类型，必须是标准布局和可平凡复制的
     * @param pack 要发送的结构体引用
     * @param time_out 超时时间（毫秒），默认5ms
     * @return true 发送请求已提交
     * 
     * @details 模板函数，用于发送任意符合要求的结构体。
     *          使用static_assert确保结构体满足传输要求。
     * 
     * 结构体要求：
     * 1. 标准布局（std::is_standard_layout）
     * 2. 可平凡复制（std::is_trivial）
     * 3. 不包含指针或引用成员
     * 
     * @note 常用于发送Arm_t等机器人数据结构
     * @see Arm_t 机器人数据结构
     */
    template <typename T>
    bool send_struct(const T& pack, unsigned int time_out=5) {
        static_assert(std::is_standard_layout<T>::value, "结构体不是标准构型");
        static_assert(std::is_trivial<T>::value, "数据包必须是可复制的");

        constexpr int pack_size = sizeof(T);

        send(reinterpret_cast<const uint8_t*>(&pack), pack_size, time_out);
        return true;
    }

    /**
     * @brief 处理一次USB事件
     * 
     * @details 调用libusb的事件处理函数，处理挂起的USB事件。
     *          通常由专用线程循环调用，用于：
     *          1. 处理接收到的数据
     *          2. 检测设备热插拔
     *          3. 处理传输完成事件
     * 
     * @note 设置50ms超时，避免长时间阻塞
     * @warning 必须在打开设备后调用
     */
    void process_once();

private:
    /**
     * @brief 热插拔事件处理函数
     * 
     * @param event 热插拔事件类型
     * 
     * @details 处理USB设备的热插拔事件：
     *          - LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT: 设备断开
     *          - LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED: 设备插入
     * 
     * @note 在libusb的事件线程中调用
     */
    void on_hotplug(libusb_hotplug_event event);

    uint16_t last_vid;      /**< 最后连接的设备供应商ID */
    uint16_t last_pid;      /**< 最后连接的设备产品ID */
    int interfaces_num;     /**< USB接口数量 */

    /** @brief 数据接收回调函数 */
    std::function<void(const uint8_t* data, int size)> cdc_recv_cb;

    std::atomic_bool _disconnected;     /**< 设备断开标志 */
    std::atomic_bool _need_reconnected; /**< 需要重连标志 */
    std::atomic_bool _handling_events;  /**< 正在处理事件标志 */

    uint8_t cdc_rx_buffer[2048];        /**< 接收缓冲区，2KB大小 */
    libusb_transfer* recv_transfer;     /**< 异步接收传输结构体 */
    libusb_context* ctx;                /**< libusb上下文 */
    libusb_device_handle* handle;       /**< USB设备句柄 */
    libusb_hotplug_callback_handle hotplug_handle; /**< 热插拔回调句柄 */
};

#endif /* __CDC_TRANS_H__ */