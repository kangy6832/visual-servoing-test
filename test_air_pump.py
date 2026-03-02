#!/usr/bin/env python3
"""
测试气泵控制功能的简单脚本
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
import time

class AirPumpTestNode(Node):
    def __init__(self):
        super().__init__('air_pump_test_node')
        
        # 创建参数客户端
        self.param_client = self.create_client(SetParameters, '/driver_node/set_parameters')
        self.get_param_client = self.create_client(GetParameters, '/driver_node/get_parameters')
        
        # 等待服务可用
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 set_parameters 服务...')
        
        while not self.get_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 get_parameters 服务...')
        
        self.get_logger().info('气泵测试节点已启动')
    
    def set_air_pump(self, enable):
        """设置气泵状态"""
        self.get_logger().info(f"设置气泵: {'开启' if enable else '关闭'}")
        
        # 创建参数
        param = Parameter()
        param.name = 'enable_air_pump'
        param.value = ParameterValue()
        param.value.type = ParameterValue.Type.BOOL
        param.value.bool_value = enable
        
        # 创建请求
        req = SetParameters.Request()
        req.parameters = [param]
        
        # 发送请求
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            result = future.result()
            if result.results and result.results[0].successful:
                self.get_logger().info('气泵设置成功')
                return True
            else:
                self.get_logger().error(f'气泵设置失败: {result.results[0].reason if result.results else "未知错误"}')
                return False
        else:
            self.get_logger().error('调用气泵服务失败')
            return False
    
    def get_air_pump_status(self):
        """获取气泵状态"""
        req = GetParameters.Request()
        req.names = ['enable_air_pump']
        
        future = self.get_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            result = future.result()
            if result.values:
                status = result.values[0].bool_value
                self.get_logger().info(f"当前气泵状态: {'开启' if status else '关闭'}")
                return status
            else:
                self.get_logger().error('获取气泵状态失败')
                return None
        else:
            self.get_logger().error('调用获取气泵状态服务失败')
            return None
    
    def test_air_pump_sequence(self):
        """测试完整的气泵控制序列"""
        self.get_logger().info('开始气泵控制测试')
        
        # 测试1: 开启气泵
        success = self.set_air_pump(True)
        if not success:
            return False
        
        time.sleep(1.0)
        
        # 验证开启状态
        status = self.get_air_pump_status()
        if status is None or not status:
            self.get_logger().error('气泵开启验证失败')
            return False
        
        # 测试2: 关闭气泵
        success = self.set_air_pump(False)
        if not success:
            return False
        
        time.sleep(0.5)
        
        # 验证关闭状态
        status = self.get_air_pump_status()
        if status is None or status:
            self.get_logger().error('气泵关闭验证失败')
            return False
        
        self.get_logger().info('气泵控制测试完成')
        return True
    
    def test_retry_mechanism(self):
        """测试重试机制"""
        self.get_logger().info('测试重试机制')
        
        # 模拟多次设置
        for i in range(3):
            self.get_logger().info(f'第 {i+1} 次设置气泵开启')
            success = self.set_air_pump(True)
            if success:
                break
            time.sleep(0.1)
        
        # 验证最终状态
        status = self.get_air_pump_status()
        if status:
            self.get_logger().info('重试机制测试成功')
            return True
        else:
            self.get_logger().error('重试机制测试失败')
            return False

def main():
    rclpy.init()
    
    try:
        node = AirPumpTestNode()
        
        # 基本功能测试
        if node.test_air_pump_sequence():
            node.get_logger().info('✓ 基本功能测试通过')
        else:
            node.get_logger().error('✗ 基本功能测试失败')
        
        time.sleep(1.0)
        
        # 重试机制测试
        if node.test_retry_mechanism():
            node.get_logger().info('✓ 重试机制测试通过')
        else:
            node.get_logger().error('✗ 重试机制测试失败')
        
    except Exception as e:
        print(f'测试过程中发生异常: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
