# ===========================================
# PICO ROS2 Bridge Docker Image
# 独立环境: PICO 设备数据 → ROS2 话题
# ===========================================
#
# 基础镜像说明:
#   osrf/ros:humble-desktop 基于 Ubuntu 22.04 LTS
#   已预装 ROS2 Humble 完整桌面版
#
# XRoboToolkit 组件:
#   1. PC-Service (.deb): 接收 PICO 设备 gRPC 数据
#      - 安装位置: /opt/apps/roboticsservice/
#      - 启动脚本: /opt/apps/roboticsservice/runService.sh
#
#   2. xrobotoolkit_sdk (Python): 访问追踪数据的 Python 接口
#      - 基于 PXREARobotSDK (C++) 构建
#      - 需要从源码编译安装
#
# 版本信息 (截至 2025年6月):
#   - PC-Service: v1.0.0 (2025-06-10 发布)
#   - Python SDK: 从 GitHub 源码构建
#
# ===========================================

FROM osrf/ros:humble-desktop

LABEL maintainer="zhuliang"
LABEL description="PICO XR devices to ROS2 bridge"
LABEL version="1.0.0"

# 避免交互式安装
ENV DEBIAN_FRONTEND=noninteractive

# ===== 系统依赖 =====
RUN apt-get update && apt-get install -y \
    # 基础工具
    git \
    wget \
    curl \
    vim \
    # Python 构建依赖
    python3-pip \
    python3-dev \
    python3-pybind11 \
    # C++ 构建工具
    build-essential \
    cmake \
    # 网络工具
    net-tools \
    iputils-ping \
    # ROS2 DDS
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ===== XRoboToolkit PC-Service =====
#
# 官方 GitHub: https://github.com/XR-Robotics/XRoboToolkit-PC-Service
#
# 版本: v1.0.0 (2025-06-10)
# 文件名: XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
#
# 安装后:
#   - 程序位置: /opt/apps/roboticsservice/
#   - 启动服务: /opt/apps/roboticsservice/runService.sh
#   - 或双击桌面图标 xrobotoolkit-pc-service
#
# 注意: deb 包的 post-install 脚本需要 xdg-utils 来安装桌面图标
#
ARG XROBOTOOLKIT_VERSION=1.0.0
RUN apt-get update && apt-get install -y xdg-utils \
    && rm -rf /var/lib/apt/lists/*
RUN wget -q https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v${XROBOTOOLKIT_VERSION}/XRoboToolkit_PC_Service_${XROBOTOOLKIT_VERSION}_ubuntu_22.04_amd64.deb \
    -O /tmp/xrobotoolkit-pc-service.deb \
    && dpkg -i /tmp/xrobotoolkit-pc-service.deb || apt-get update && apt-get install -f -y \
    && rm /tmp/xrobotoolkit-pc-service.deb

# ===== XRoboToolkit Python SDK (xrobotoolkit_sdk) =====
#
# 官方 GitHub:
#   - Pybind: https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind
#   - PC-Service: https://github.com/XR-Robotics/XRoboToolkit-PC-Service
#
# 官方构建步骤 (参考 README.md):
#   1. 克隆 XRoboToolkit-PC-Service-Pybind 仓库
#   2. 克隆 XRoboToolkit-PC-Service 仓库
#   3. 进入 PXREARobotSDK 目录，执行 bash build.sh
#   4. 复制头文件 (PXREARobotSDK.h, nlohmann/) 和库文件 (libPXREARobotSDK.so)
#   5. 执行 pip install pybind11 && python setup.py install
#
# 最终安装包名: xrobotoolkit_sdk
#
WORKDIR /tmp/sdk_build

# 1. 克隆 Pybind 仓库
RUN git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git pybind

# 2. 克隆 PC-Service 仓库
RUN git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git pc-service

# 3. 构建 C++ SDK (使用官方 build.sh 脚本)
#    build.sh 会创建 build 目录并执行 cmake + make
RUN cd pc-service/RoboticsService/PXREARobotSDK \
    && bash build.sh

# 4. 复制头文件和库文件到 Pybind 目录
#    - PXREARobotSDK.h: 核心 SDK 头文件
#    - nlohmann/: JSON 库头文件
#    - libPXREARobotSDK.so: 编译后的动态链接库
RUN mkdir -p pybind/include pybind/lib \
    && cp pc-service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h pybind/include/ \
    && cp -r pc-service/RoboticsService/PXREARobotSDK/nlohmann pybind/include/ \
    && cp pc-service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so pybind/lib/

# 5. 安装 Python SDK
#    安装后可通过 import xrobotoolkit_sdk 使用
RUN cd pybind \
    && pip3 install pybind11 \
    && python3 setup.py install

# 6. 复制动态库到系统路径，确保运行时可找到
RUN cp pybind/lib/libPXREARobotSDK.so /usr/local/lib/ \
    && ldconfig

# 清理构建文件
RUN rm -rf /tmp/sdk_build

WORKDIR /

# ===== Python 依赖 =====
COPY ros2_ws/src/pico_bridge/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ===== 设置 ROS2 工作空间 =====
WORKDIR /ros2_ws

# 复制 ROS2 包
COPY ros2_ws/src /ros2_ws/src

# 转换所有 Python 和 launch 文件的 Windows CRLF 为 Unix LF
RUN find /ros2_ws/src -type f \( -name "*.py" -o -name "*.launch.py" \) -exec sed -i 's/\r$//' {} \;

# 安装 ROS2 依赖
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# 构建 ROS2 包
RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install

# 再次转换 install 目录下的文件 (colcon 可能复制而非符号链接某些文件)
RUN find /ros2_ws/install -type f \( -name "*.py" -o -name "*.launch.py" \) -exec sed -i 's/\r$//' {} \;

# ===== 入口脚本 =====
COPY scripts/entrypoint.sh /entrypoint.sh
# 转换 Windows CRLF 为 Unix LF，并设置执行权限
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

# ===== 环境配置 =====
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 确保能找到 libPXREARobotSDK.so
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# 配置 bashrc，进入容器时自动加载 ROS2 环境
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# 暴露端口
# - 8800: XRoboToolkit gRPC (PICO 设备连接)
# - 7400-7410: ROS2 DDS 通信
EXPOSE 8800
EXPOSE 7400-7410/udp

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "pico_bridge", "pico_bridge.launch.py"]
