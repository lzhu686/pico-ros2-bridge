#!/bin/bash
# ===========================================
# PICO ROS2 Bridge - Git 仓库初始化脚本
# ===========================================
#
# 使用方法:
#   cd pico-ros2-bridge
#   bash scripts/init_git.sh
#
# 功能:
#   1. 初始化 Git 仓库
#   2. 创建首次提交
#   3. 显示后续步骤提示
#

set -e

echo "==========================================="
echo "  PICO ROS2 Bridge - Git 初始化"
echo "==========================================="

# 检查是否已经是 Git 仓库
if [ -d ".git" ]; then
    echo "[WARN] 当前目录已是 Git 仓库"
    echo "       如需重新初始化，请先删除 .git 目录"
    exit 1
fi

# 检查是否在正确目录
if [ ! -f "Dockerfile" ] || [ ! -f "docker-compose.yml" ]; then
    echo "[ERROR] 请在 pico-ros2-bridge 目录下运行此脚本"
    exit 1
fi

echo ""
echo "[1/3] 初始化 Git 仓库..."
git init

echo ""
echo "[2/3] 添加文件到暂存区..."
git add .

echo ""
echo "[3/3] 创建首次提交..."
git commit -m "Initial commit: PICO ROS2 Bridge Docker module

功能:
- 接收 PICO 头显、手柄、Motion Tracker 的 6DoF 位姿数据
- 发布为标准 ROS2 话题
- 支持模拟模式 (无 PICO 设备时用于测试)

ROS2 话题:
- /pico/hmd/pose (头显位姿)
- /pico/controller/left/pose, /pico/controller/right/pose (手柄位姿)
- /pico/tracker/left_wrist, /pico/tracker/right_wrist (腕部 Tracker)
- /pico/tracker/left_elbow, /pico/tracker/right_elbow (肘部 Tracker)

依赖:
- 基础镜像: osrf/ros:humble-desktop (Ubuntu 22.04 + ROS2 Humble)
- XRoboToolkit PC-Service v1.0.0 (2025-06-10)
- xrobotoolkit_sdk (Python SDK, 从源码构建)"

echo ""
echo "==========================================="
echo "  初始化完成!"
echo "==========================================="
echo ""
echo "后续步骤:"
echo ""
echo "  1. 在 GitHub 创建新仓库: pico-ros2-bridge"
echo "     https://github.com/new"
echo ""
echo "  2. 关联远程仓库并推送:"
echo "     git remote add origin https://github.com/YOUR_USERNAME/pico-ros2-bridge.git"
echo "     git branch -M main"
echo "     git push -u origin main"
echo ""
echo "  或使用 GitHub CLI (需先安装 gh):"
echo "     gh repo create pico-ros2-bridge --public --source=. --push"
echo ""
echo "==========================================="
