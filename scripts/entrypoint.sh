#!/bin/bash
set -e

# 设置 ROS2 环境
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "=========================================="
echo "  PICO ROS2 Bridge"
echo "=========================================="
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo ""

# 检查 XRoboToolkit PC-Service
if [ -f "/opt/apps/roboticsservice/runService.sh" ]; then
    echo "[OK] XRoboToolkit PC-Service 已安装"
    echo "     位置: /opt/apps/roboticsservice/"
else
    echo "[WARN] XRoboToolkit PC-Service 未找到"
fi

# 检查 xrobotoolkit_sdk Python 模块
python3 -c "import xrobotoolkit_sdk; print('[OK] xrobotoolkit_sdk 模块可用')" 2>/dev/null || \
    echo "[WARN] xrobotoolkit_sdk 模块不可用，将使用模拟模式"

# 检查 libPXREARobotSDK.so
if ldconfig -p | grep -q libPXREARobotSDK; then
    echo "[OK] libPXREARobotSDK.so 已加载"
else
    echo "[WARN] libPXREARobotSDK.so 未找到"
fi

echo ""
echo "=========================================="
echo "  查看话题数据"
echo "=========================================="
echo "# 列出所有 PICO 话题:"
echo "docker exec pico-ros2-bridge ros2 topic list | grep pico"
echo ""
echo "# 查看头显位姿:"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/hmd/pose"
echo ""
echo "# 查看手柄位姿:"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/controller/left/pose"
echo ""
echo "=========================================="
echo "  重要提示"
echo "=========================================="
echo "1. 确保 XRoboToolkit PC-Service 正在运行:"
echo "   docker exec pico-ros2-bridge /opt/apps/roboticsservice/runService.sh"
echo "2. 在 PICO 上安装 XRoboToolkit Client APK"
echo "3. 确保 PICO 与 PC 在同一局域网"
echo "=========================================="
echo ""

exec "$@"
