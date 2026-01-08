#!/bin/bash
set -e

# 设置 ROS2 环境
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              PICO ROS2 Bridge - 启动信息                      ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# ============================================================
# PICO 连接信息
# ============================================================
echo "┌──────────────────────────────────────────────────────────────┐"
echo "│  📡 PICO 头显连接                                            │"
echo "├──────────────────────────────────────────────────────────────┤"
echo "│                                                              │"
echo "│  在宿主机上查询 IP 地址:                                      │"
echo "│    Linux/Mac:  hostname -I  或  ifconfig                     │"
echo "│    Windows:    ipconfig                                      │"
echo "│                                                              │"
echo "│  选择与 PICO 头显在同一网段的 IP (通常是 192.168.x.x)         │"
echo "│  端口: 63901                                                  │"
echo "│                                                              │"
echo "└──────────────────────────────────────────────────────────────┘"
echo ""

# ============================================================
# 系统信息
# ============================================================
echo "┌──────────────────────────────────────────────────────────────┐"
echo "│  ⚙️  系统配置                                                 │"
echo "├──────────────────────────────────────────────────────────────┤"
echo "│  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo "│  RMW: ${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
echo "└──────────────────────────────────────────────────────────────┘"
echo ""

# 检查 XRoboToolkit PC-Service
if [ -f "/opt/apps/roboticsservice/runService.sh" ]; then
    echo "[✓] XRoboToolkit PC-Service 已安装"
else
    echo "[✗] XRoboToolkit PC-Service 未找到"
fi

# 检查 xrobotoolkit_sdk Python 模块
python3 -c "import xrobotoolkit_sdk; print('[✓] xrobotoolkit_sdk 模块可用')" 2>/dev/null || \
    echo "[✗] xrobotoolkit_sdk 模块不可用"

# 检查 libPXREARobotSDK.so
if ldconfig -p | grep -q libPXREARobotSDK; then
    echo "[✓] libPXREARobotSDK.so 已加载"
else
    echo "[✗] libPXREARobotSDK.so 未找到"
fi

echo ""
echo "=========================================="
echo "  查看话题数据"
echo "=========================================="
echo ""
echo "# 列出所有 PICO 话题:"
echo "docker exec pico-ros2-bridge ros2 topic list | grep pico"
echo ""
echo "# 查看头显姿态 (200Hz):"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/hmd/pose"
echo ""
echo "# 查看手柄姿态:"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/controller/left/pose"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/controller/right/pose"
echo ""
echo "# 查看手柄按键/摇杆:"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/controller/left/joy"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/controller/right/joy"
echo ""
echo "# 查看追踪器姿态:"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/tracker/left_wrist/pose"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/tracker/right_wrist/pose"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/tracker/left_ankle/pose"
echo "docker exec pico-ros2-bridge ros2 topic echo /pico/tracker/right_ankle/pose"
echo ""
echo "=========================================="
echo "  重要提示"
echo "=========================================="
echo ""
echo "1. 确保 PICO 头显和运行 Docker 的电脑在同一局域网"
echo "2. PICO 头显上打开 XRoboToolkit App，输入上述 IP 和端口连接"
echo "3. 连接成功后，ROS2 话题会自动开始发布数据"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

exec "$@"
