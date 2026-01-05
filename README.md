# PICO ROS2 Bridge

将 PICO XR 设备数据桥接到 ROS2 话题的独立 Docker 环境。

## 项目定位

这是一个**独立的 Docker 模块**，专注于：
- 接收 PICO 头显、手柄、Motion Tracker 的 6DoF 位姿数据
- 发布为标准 ROS2 话题
- 与其他 ROS2 系统 (如 Manus 手套、机械臂控制) 解耦

**后期集成路线：** 本 Docker 测试通过后，可与 `wuji-system-docker` 等其他模块通过 ROS2 话题通信。

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
│  • wuji-system-docker (Manus 手套 + 运动重定向)                │
│  • tianji_arm_control (天机机械臂控制)                         │
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

### 1. 构建 Docker 镜像

```bash
cd pico-ros2-bridge
docker compose build
```

### 2. 启动服务

```bash
# 正常模式 (需要 PICO 设备连接)
docker compose up

# 模拟模式 (无 PICO 设备，用于测试 ROS2 话题)
docker compose --profile simulation up pico-bridge-sim
```

### 3. PICO 端配置

1. 在 PICO 头显上安装 **XRoboToolkit Client APK**
2. 确保 PICO 与 PC 在**同一局域网**
3. 打开 XRoboToolkit Client，输入 PC 的 IP 地址
4. 点击连接，等待状态变为 "Connected"

### 4. 验证数据

```bash
# 进入容器
docker compose exec pico-bridge bash

# 查看所有话题
ros2 topic list

# 查看头显数据
ros2 topic echo /pico/hmd/pose

# 查看左腕追踪器
ros2 topic echo /pico/tracker/left_wrist

# 查看话题频率
ros2 topic hz /pico/tracker/left_wrist
```

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

### 与 wuji-system-docker 集成

测试本 Docker 通过后，可在 `wuji-system-docker` 中添加：

```yaml
# wuji-system-docker/docker-compose.yml
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
