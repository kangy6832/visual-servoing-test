#!/bin/bash

# 测试通信数据流的脚本

echo "=== 通信测试脚本 ==="
echo "1. 检查节点状态"
source install/setup.bash

echo "当前运行的节点："
ros2 node list | grep -E "(driver_node|controller_node)"

echo ""
echo "2. 检查话题连接状态"
echo "发布者数量："
ros2 topic info /myjoints_state | grep "Publisher count"
echo "订阅者数量："
ros2 topic info /myjoints_state | grep "Subscription count"

echo ""
echo "3. 检查话题频率（如果有数据流）"
echo "检查 /myjoints_state 频率："
timeout 3 ros2 topic hz /myjoints_state 2>/dev/null || echo "无数据流或消息类型错误"

echo ""
echo "4. 检查节点日志"
echo "检查 controller_node 日志："
ros2 log info controller_node 2>/dev/null | tail -5 || echo "无法获取日志"

echo ""
echo "5. 检查 driver_node 日志"
echo "检查 driver_node 日志："
ros2 log info driver_node 2>/dev/null | tail -5 || echo "无法获取日志"

echo ""
echo "=== 测试完成 ==="
