# PICO ROS2 Bridge

将 PICO XR 设备数据桥接到 ROS2 话题的独立 Docker 环境。

Key project and APK notes are provided in both Chinese and English.

## 项目说明

该项目是我在 Wuji 远程操作/数据采集相关工作中独立完成的 PICO XR 调用与 ROS2 桥接子模块，用于把头显、手柄和 Motion Tracker 数据整理为标准 ROS2 话题。

这个仓库更偏向一个可复用的小工具和使用说明：如果你想了解“如何从 PICO 获取数据，并把它接入 ROS2”，可以通过这里复现基础流程。公司开源的 `wuji-hand-teleop` 等仓库会提供更完整的 PICO 与机械臂/灵巧手集成能力；本仓库与 `wuji-hand-teleop` 没有代码依赖关系，也不包含公司内部的完整系统集成、控制策略、私有部署和镜像分发细节。

## Project Note

This repository contains the PICO XR data access and ROS2 bridge submodule that I implemented independently during Wuji teleoperation/data-collection work. It converts PICO HMD, controller, and Motion Tracker data into standard ROS2 topics.

The goal is to provide a reusable reference tool for people who want to understand how to get data from PICO devices and feed it into ROS2. More complete robot/hand teleoperation integrations are expected to live in company-maintained open-source repositories such as `wuji-hand-teleop`. This repository has no code dependency relationship with `wuji-hand-teleop`, and it does not include internal full-system integration, control policies, private deployment details, or private image distribution.

## 项目定位

这是一个**独立的 Docker 模块**，专注于：
- 接收 PICO 头显、手柄、Motion Tracker 的 6DoF 位姿数据
- 发布为标准 ROS2 话题
- 与其他 ROS2 系统 (如 Manus 手套、机械臂控制) 解耦

**后期集成路线：** 本 Docker 测试通过后，可通过 ROS2 话题与手部遥操作、机械臂控制、运动重定向等模块解耦集成。

In short, this module focuses on PICO-to-ROS2 data bridging. Downstream hand teleoperation, arm control, and retargeting modules can subscribe to the ROS2 topics exposed here.

## 支持设备

| 设备 | 数据类型 | 说明 |
|------|----------|------|
| PICO 头显 (HMD) | 6DoF 位姿 | 头部追踪，用于立体视觉显示 |
| PICO 手柄 | 6DoF + 按键 | 左/右手柄位姿和按键状态 |
| PICO Motion Tracker | 6DoF 位姿 | 最多 4 个，用于腕部/肘部追踪 |

## ROS2 话题输出

### 头显与手柄

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/pico/hmd/pose` | PoseStamped | 90Hz | 头显位姿 |
| `/pico/controller/left/pose` | PoseStamped | 90Hz | 左手柄位姿 |
| `/pico/controller/right/pose` | PoseStamped | 90Hz | 右手柄位姿 |
| `/pico/controller/left/joy` | Joy | 90Hz | 左手柄按键 |
| `/pico/controller/right/joy` | Joy | 90Hz | 右手柄按键 |

### Motion Tracker (遥操作用)

| 话题 | 类型 | 频率 | 用途 |
|------|------|------|------|
| `/pico/tracker/left_wrist` | PoseStamped | 200Hz | 左臂末端位姿 (Tracker #0) |
| `/pico/tracker/right_wrist` | PoseStamped | 200Hz | 右臂末端位姿 (Tracker #1) |
| `/pico/tracker/left_elbow` | PoseStamped | 200Hz | 左臂肘部约束 (Tracker #2) |
| `/pico/tracker/right_elbow` | PoseStamped | 200Hz | 右臂肘部约束 (Tracker #3) |

**Tracker 佩戴位置：**
```
Tracker #0 → 左手腕  → /pico/tracker/left_wrist  → 控制天机左臂末端 6DoF
Tracker #1 → 右手腕  → /pico/tracker/right_wrist → 控制天机右臂末端 6DoF
Tracker #2 → 左上臂  → /pico/tracker/left_elbow  → 提供左臂 Elbow Hint
Tracker #3 → 右上臂  → /pico/tracker/right_elbow → 提供右臂 Elbow Hint
```

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         用户端                                   │
│                                                                 │
│   PICO 头显          4× Motion Tracker          PICO 手柄       │
│   (HMD)              (腕部×2 + 肘部×2)          (可选)          │
│      │                      │                      │            │
└──────┼──────────────────────┼──────────────────────┼────────────┘
       │                      │                      │
       │         WiFi / 局域网 (gRPC)                │
       │                      │                      │
┌──────▼──────────────────────▼──────────────────────▼────────────┐
│                   pico-ros2-bridge Docker                       │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  XRoboToolkit PC-Service                                │   │
│  │  /opt/apps/roboticsservice/                             │   │
│  │  • 接收 PICO 设备 gRPC 数据流                            │   │
│  │  • 解析头显、手柄、Tracker 数据                          │   │
│  └─────────────────────────────────────────────────────────┘   │
│                            │                                    │
│                            │ xrobotoolkit_sdk (Python)          │
│                            ▼                                    │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  pico_bridge_node (ROS2)                                │   │
│  │                                                         │   │
│  │  发布话题:                                               │   │
│  │  • /pico/hmd/pose                                       │   │
│  │  • /pico/controller/{left,right}/pose                   │   │
│  │  • /pico/tracker/{left_wrist,right_wrist,...}          │   │
│  └─────────────────────────────────────────────────────────┘   │
│                            │                                    │
└────────────────────────────┼────────────────────────────────────┘
                             │ ROS2 DDS
                             ▼
┌────────────────────────────────────────────────────────────────┐
│                    其他 ROS2 系统                               │
│                                                                │
│  • wuji-hand-teleop / hand teleoperation modules               │
│  • robot arm control / motion retargeting modules              │
│  • ...                                                         │
└────────────────────────────────────────────────────────────────┘
```

## 依赖说明

### 基础镜像

```dockerfile
FROM osrf/ros:humble-desktop
```

- 基于 **Ubuntu 22.04 LTS**
- 预装 **ROS2 Humble** 完整桌面版
- 无需从裸 Ubuntu 构建 ROS2，节省 1-2 小时构建时间

### XRoboToolkit PC-Service

| 项目 | 说明 |
|------|------|
| 版本 | v1.0.0 (**2025-06-10** 发布) |
| 格式 | `.deb` 安装包 (仅支持 Ubuntu 22.04 x86_64) |
| 文件名 | `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb` |
| 安装位置 | `/opt/apps/roboticsservice/` |
| 启动脚本 | `/opt/apps/roboticsservice/runService.sh` |
| 来源 | [GitHub Releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases) |

### xrobotoolkit_sdk Python 绑定

| 项目 | 说明 |
|------|------|
| 来源 | [XRoboToolkit-PC-Service-Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind) |
| 底层 SDK | PXREARobotSDK (C++) |
| 安装方式 | 从源码构建 (需要先编译 PXREARobotSDK) |
| Python 包名 | `xrobotoolkit_sdk` |

## xrobotoolkit_sdk API 参考

官方 API 文档，参考 [examples/](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/tree/main/examples)。

### 初始化与清理

```python
import xrobotoolkit_sdk as xrt

xrt.init()   # 初始化 SDK 连接
xrt.close()  # 关闭 SDK 连接
```

### 位姿获取 (返回 [x, y, z, qx, qy, qz, qw])

```python
xrt.get_headset_pose()           # 头显位姿
xrt.get_left_controller_pose()   # 左手柄位姿
xrt.get_right_controller_pose()  # 右手柄位姿
```

### 手柄输入

```python
# 模拟输入 (float 0-1)
xrt.get_left_trigger()   # 左扳机
xrt.get_right_trigger()  # 右扳机
xrt.get_left_grip()      # 左握把
xrt.get_right_grip()     # 右握把

# 摇杆 (返回 [x, y])
xrt.get_left_axis()      # 左摇杆
xrt.get_right_axis()     # 右摇杆

# 按键状态 (bool)
xrt.get_A_button()       # A 键 (右手柄)
xrt.get_B_button()       # B 键 (右手柄)
xrt.get_X_button()       # X 键 (左手柄)
xrt.get_Y_button()       # Y 键 (左手柄)
```

### Motion Tracker (独立追踪器)

```python
xrt.num_motion_data_available()        # 可用追踪器数量
xrt.get_motion_tracker_pose()          # 追踪器位姿数组 (Nx7)
xrt.get_motion_tracker_velocity()      # 追踪器速度
xrt.get_motion_tracker_acceleration()  # 追踪器加速度
xrt.get_motion_tracker_serial_numbers() # 追踪器序列号
xrt.get_motion_timestamp_ns()          # 时间戳 (纳秒)
```

### Body Tracking (24 关节全身追踪)

需要至少 2 个 Swift 设备进行校准。

```python
xrt.is_body_data_available()           # 是否有身体数据
xrt.get_body_joints_pose()             # 24 关节位姿 (24x7 数组)
xrt.get_body_joints_velocity()         # 24 关节速度
xrt.get_body_joints_acceleration()     # 24 关节加速度
xrt.get_body_joints_timestamp()        # 各关节 IMU 时间戳
xrt.get_body_timestamp_ns()            # 身体数据时间戳
```

**24 关节 SMPL 骨骼顺序:**
```
 0: pelvis       1: left_hip      2: right_hip     3: spine1
 4: left_knee    5: right_knee    6: spine2        7: left_ankle
 8: right_ankle  9: spine3       10: left_foot    11: right_foot
12: neck        13: left_collar  14: right_collar 15: head
16: left_shoulder 17: right_shoulder 18: left_elbow 19: right_elbow
20: left_wrist  21: right_wrist  22: left_hand    23: right_hand
```

### 手部追踪

```python
xrt.get_left_hand_tracking_state()     # 左手 27x7 数组
xrt.get_right_hand_tracking_state()    # 右手 27x7 数组
xrt.get_left_hand_is_active()          # 左手是否激活
xrt.get_right_hand_is_active()         # 右手是否激活
```

### 时间戳

```python
xrt.get_time_stamp_ns()                # 当前时间戳 (纳秒)
```

## 快速开始

### 本地构建镜像（推荐）

```bash
cd pico-ros2-bridge
docker compose build

# Linux 用户 (默认)
docker compose up -d

# Windows / Mac 用户 (Docker Desktop)
docker compose --profile windows up -d
```

| 平台 | 命令 | 网络模式 | 说明 |
|------|------|----------|------|
| **Linux** | `docker compose up -d` | host | 默认配置，最佳性能 |
| Windows/Mac | `docker compose --profile windows up -d` | 端口映射 | Docker Desktop 需要端口映射 |

### 查看话题数据

```bash
# 列出所有 PICO 话题 (Linux)
docker exec pico-ros2-bridge ros2 topic list | grep pico

# 列出所有 PICO 话题 (Windows/Mac)
docker exec pico-ros2-bridge-win ros2 topic list | grep pico

# 查看头显位姿
docker compose exec pico-bridge bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /pico/hmd/pose"

# 查看左腕追踪器
docker compose exec pico-bridge bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /pico/tracker/left_wrist"

# 查看话题频率
docker compose exec pico-bridge bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /pico/tracker/left_wrist"
```

### 查看连接 IP 地址

PICO 头显需要输入运行 Docker 的电脑的局域网 IP 地址进行连接。

**查询宿主机 IP:**
```bash
# Linux / Mac
hostname -I
# 或
ifconfig

# Windows
ipconfig
```

选择与 PICO 头显在同一网段的 IP (通常是 `192.168.x.x` 或 `10.x.x.x`)，端口为 **63901**。

### PICO 端配置

**前置条件:**
- PICO 4 Ultra 头显
- User OS > 5.12
- 已开启开发者模式 ([开启方法](https://developer-global.pico-interactive.com/document/doc/enable-developer-mode))
- 企业版权限 + VST 相机权限 (用于双目视频流)

**安装 XRoboToolkit Client APK:**

```bash
# 使用本仓库提供的最新 APK
adb install -r -g apk/v1.4.apk

# 如需回退旧版，可使用 v1.3 local 版本
adb install -r -g apk/v1.3local.apk
```

| 资源 | 说明 |
|------|------|
| 本仓库 APK | [apk/v1.4.apk](apk/) (**最新版本**) / [apk/v1.3local.apk](apk/) (旧版 local 坐标系) |
| Unity 源码参考 | [lzhu686/XRoboToolkit-Unity-Client](https://github.com/lzhu686/XRoboToolkit-Unity-Client) |

**版本说明：** 这里的 `v1.4` 指本仓库中 APK 文件的迭代版本。当前 APK 的 Android Manifest 仍显示 `versionName=1.0.0`、`versionCode=1`，因此请优先以文件名和本文档说明区分版本。

**视觉方案说明：** 本仓库提供的 APK 在官方 XRoboToolkit Client 的数据获取能力基础上，额外整理了一个面向低成本双目 RGB 立体视觉的参考配置。相较于常见的官方/高端双目方案（例如 ZED 等深度相机），这个方案尝试使用更便宜的 USB 双目 RGB 相机完成 VR 端立体视觉显示。当前 `v1.4.apk` 内置 `assets/video_source.yml`，配置了 USB 双目 RGB 相机示例（如 2560x720@60fps、ADB 约 23Mbps、WiFi 约 11Mbps）以及 16:9 双目画面的显示比例参数。这个方案仅供低成本遥操作视觉链路参考；如果你已经有官方推荐或更高端的 ZED 等深度/双目相机方案，也可以继续使用官方方案做数据获取。

**APK notes (English):** `v1.4` is the repository file revision, while the Android Manifest still reports `versionName=1.0.0` and `versionCode=1`. The bundled APK keeps the official XRoboToolkit data-access workflow and adds a reference configuration for a lower-cost stereo RGB setup. Compared with common official/high-end stereo setups such as ZED depth cameras, this reference path targets a cheaper USB stereo RGB camera for VR-side stereo display. The included `assets/video_source.yml` contains a 2560x720@60fps camera example, ADB/WiFi bitrate presets, and 16:9 stereo rendering parameters. ZED or other official/high-end camera setups can still be used if available.

**坐标系模式：推荐使用 Local（本地坐标系）更稳定可靠。Global 模式仅在需要多设备空间对齐时使用。**

**连接步骤:**
1. 确保 PICO 与 PC 在**同一局域网**
2. 打开 XRoboToolkit Client，输入 PC 的 IP 地址
3. 点击连接，等待状态变为 "Connected"

## 配置参数

### Launch 参数

```bash
ros2 launch pico_bridge pico_bridge.launch.py \
    publish_rate:=200.0 \          # 发布频率 (Hz)
    frame_id:=pico_world \         # TF 父坐标系
    enable_tf:=true \              # 是否发布 TF
    enable_hmd:=true \             # 启用头显
    enable_controllers:=true \     # 启用手柄
    enable_trackers:=true \        # 启用追踪器
    num_trackers:=4 \              # 追踪器数量
    simulation_mode:=false         # 模拟模式
```

### Tracker 角色映射

可通过参数调整 Tracker ID 与角色的对应关系：

```bash
ros2 launch pico_bridge pico_bridge.launch.py \
    tracker_0_role:=left_wrist \
    tracker_1_role:=right_wrist \
    tracker_2_role:=left_elbow \
    tracker_3_role:=right_elbow
```

## 项目结构

```
pico-ros2-bridge/
├── Dockerfile                 # Docker 镜像 (详细注释)
├── docker-compose.yml         # Docker Compose 配置
├── README.md                  # 本文档
├── .gitignore
├── .gitattributes             # 跨平台行尾配置
├── apk/                       # PICO 客户端 APK
│   ├── v1.4.apk                 # 最新 PICO 客户端 APK
│   └── v1.3local.apk             # 旧版 local 坐标系 APK
├── scripts/
│   └── entrypoint.sh          # 容器入口脚本
├── logs/                      # 运行日志
└── ros2_ws/
    └── src/
        └── pico_bridge/       # ROS2 包
            ├── CMakeLists.txt
            ├── package.xml
            ├── requirements.txt
            ├── pico_bridge/
            │   ├── __init__.py
            │   └── pico_bridge_node.py
            ├── launch/
            │   └── pico_bridge.launch.py
            └── config/
                └── pico_bridge.yaml
```

## 与其他系统集成

### 与其他 ROS2 系统集成

测试本 Docker 通过后，可在其他 ROS2 系统的 Docker Compose 中添加：

```yaml
# docker-compose.yml
services:
  pico-bridge:
    build: ../pico-ros2-bridge
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
```

### 话题订阅示例

```python
# 在其他节点中订阅 Tracker 数据
from geometry_msgs.msg import PoseStamped

self.left_wrist_sub = self.create_subscription(
    PoseStamped,
    '/pico/tracker/left_wrist',
    self.left_wrist_callback,
    10
)
```

## 故障排除

### PICO 无法连接

1. 确认 PICO 和 PC 在同一局域网 (相同 WiFi)
2. 检查防火墙是否开放端口 **8800**
3. 在 PICO 上重新输入正确的 PC IP 地址
4. 查看容器日志: `docker compose logs -f`

### 话题无数据

1. 检查 XRoboToolkit Client 是否显示 "Connected"
2. 使用模拟模式测试 ROS2 话题是否正常
3. 确认追踪器已在 PICO 系统中正确配对

### Docker 构建失败

1. 检查网络连接 (需要下载 GitHub 资源)
2. 如果 deb 下载失败，手动下载后放入项目目录，修改 Dockerfile 使用 `COPY`

## 参考资源

### XRoboToolkit 官方

- [XRoboToolkit 官网](https://xr-robotics.github.io/)
- [XRoboToolkit GitHub](https://github.com/XR-Robotics)
- [PC-Service 仓库](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)
- [PC-Service Releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases)
- [PC-Service-Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind)
- [Unity-Client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client)
- [Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python)

### 相关项目

- [TWIST2](https://github.com/amazon-far/TWIST2) - 人形机器人数据采集系统
- [GMR](https://github.com/YanjieZe/GMR) - 通用运动重定向

### 文档与论文

- [XRoboToolkit 论文](https://arxiv.org/abs/2508.00097)
- [PICO Developer](https://developer.picoxr.com/news/xrobotoolkit/)
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)

## 许可证

MIT License
