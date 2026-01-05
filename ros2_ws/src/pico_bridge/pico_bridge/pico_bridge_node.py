#!/usr/bin/env python3
"""
PICO ROS2 Bridge Node
将 PICO 头显、手柄、追踪器数据发布为 ROS2 话题

基于 XRoboToolkit 官方 SDK:
- GitHub: https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind
- 参考: https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python

支持设备:
- PICO 头显 (HMD): 头部 6DoF
- PICO 手柄: 左/右手柄 6DoF + 按键 (trigger, grip, buttons)
- PICO Motion Tracker: 独立追踪器 (通过序列号索引)
- PICO Body Tracking: 全身 24 关节追踪 (需要至少 2 个 Swift 设备)

ROS2 话题输出:
- /pico/hmd/pose                  : 头显位姿
- /pico/controller/left/pose      : 左手柄位姿
- /pico/controller/right/pose     : 右手柄位姿
- /pico/controller/left/joy       : 左手柄按键
- /pico/controller/right/joy      : 右手柄按键
- /pico/tracker/left_wrist        : 左腕追踪器
- /pico/tracker/right_wrist       : 右腕追踪器
- /pico/tracker/left_elbow        : 左肘追踪器
- /pico/tracker/right_elbow       : 右肘追踪器

xrobotoolkit_sdk 官方 API (参考 examples/):
初始化:
- xrt.init()                      : 初始化 SDK
- xrt.close()                     : 关闭 SDK

位姿获取 (返回 [x, y, z, qx, qy, qz, qw]):
- xrt.get_headset_pose()          : 获取头显位姿
- xrt.get_left_controller_pose()  : 获取左手柄位姿
- xrt.get_right_controller_pose() : 获取右手柄位姿

手柄输入:
- xrt.get_left_trigger()          : 左扳机 (float 0-1)
- xrt.get_right_trigger()         : 右扳机 (float 0-1)
- xrt.get_left_grip()             : 左握把 (float 0-1)
- xrt.get_right_grip()            : 右握把 (float 0-1)
- xrt.get_A_button()              : A 键状态
- xrt.get_B_button()              : B 键状态
- xrt.get_X_button()              : X 键状态
- xrt.get_Y_button()              : Y 键状态
- xrt.get_left_axis()             : 左摇杆 [x, y]
- xrt.get_right_axis()            : 右摇杆 [x, y]

Motion Tracker (独立追踪器):
- xrt.num_motion_data_available() : 可用追踪器数量
- xrt.get_motion_tracker_pose()   : 追踪器位姿 (按索引)
- xrt.get_motion_tracker_velocity(): 追踪器速度
- xrt.get_motion_tracker_acceleration(): 追踪器加速度
- xrt.get_motion_tracker_serial_numbers(): 追踪器序列号
- xrt.get_motion_timestamp_ns()   : 追踪器时间戳 (纳秒)

Body Tracking (24 关节全身追踪):
- xrt.is_body_data_available()    : 是否有身体数据
- xrt.get_body_joints_pose()      : 24 关节位姿 [x,y,z,qx,qy,qz,qw]
- xrt.get_body_joints_velocity()  : 24 关节速度
- xrt.get_body_joints_acceleration(): 24 关节加速度
- xrt.get_body_joints_timestamp() : 各关节 IMU 时间戳
- xrt.get_body_timestamp_ns()     : 身体数据时间戳

手部追踪:
- xrt.get_left_hand_tracking_state(): 左手 27x7 数组
- xrt.get_right_hand_tracking_state(): 右手 27x7 数组
- xrt.get_left_hand_is_active()   : 左手是否激活
- xrt.get_right_hand_is_active()  : 右手是否激活

时间戳:
- xrt.get_time_stamp_ns()         : 当前时间戳 (纳秒)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import threading
import numpy as np

# 尝试导入 XRoboToolkit SDK
# 官方包名: xrobotoolkit_sdk
# 安装方法: 参考 https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind
HAS_XROBOTOOLKIT = False
xrobotoolkit_sdk = None

try:
    import xrobotoolkit_sdk
    HAS_XROBOTOOLKIT = True
    print("[OK] xrobotoolkit_sdk 模块已加载")
except ImportError:
    print("[WARN] xrobotoolkit_sdk 模块未安装，将使用模拟数据模式")
    print("       安装方法: https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind")


class PicoBridgeNode(Node):
    """
    PICO 设备数据桥接节点
    从 XRoboToolkit PC-Service 获取 PICO 设备数据并发布为 ROS2 话题

    数据流:
    PICO 头显 → XRoboToolkit Unity Client (APK)
              → WiFi/gRPC
              → PC-Service (/opt/apps/roboticsservice/)
              → xrobotoolkit_sdk (Python)
              → 本节点
              → ROS2 话题
    """

    # SMPL 24关节骨骼映射 (用于 Motion Tracker 全身追踪)
    # 参考: XRoboToolkit-PC-Service-Pybind README
    BODY_JOINTS = [
        'pelvis', 'left_hip', 'right_hip', 'spine1',
        'left_knee', 'right_knee', 'spine2', 'left_ankle',
        'right_ankle', 'spine3', 'left_foot', 'right_foot',
        'neck', 'left_collar', 'right_collar', 'head',
        'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
        'left_wrist', 'right_wrist', 'left_hand', 'right_hand'
    ]

    # Tracker 角色到 SMPL 关节的映射
    TRACKER_TO_JOINT = {
        'left_wrist': 20,   # SMPL joint index for left_wrist
        'right_wrist': 21,  # SMPL joint index for right_wrist
        'left_elbow': 18,   # SMPL joint index for left_elbow
        'right_elbow': 19,  # SMPL joint index for right_elbow
    }

    def __init__(self):
        super().__init__('pico_bridge_node')

        # ===== 参数声明 =====
        self.declare_parameter('publish_rate', 90.0)  # XRoboToolkit 原生 90Hz
        self.declare_parameter('frame_id', 'pico_world')
        self.declare_parameter('enable_tf', True)
        self.declare_parameter('enable_hmd', True)
        self.declare_parameter('enable_controllers', True)
        self.declare_parameter('enable_trackers', True)
        self.declare_parameter('enable_body_tracking', False)  # 全身24关节追踪
        self.declare_parameter('num_trackers', 4)
        self.declare_parameter('simulation_mode', False)

        # Tracker 角色映射参数
        self.declare_parameter('tracker_0_role', 'left_wrist')
        self.declare_parameter('tracker_1_role', 'right_wrist')
        self.declare_parameter('tracker_2_role', 'left_elbow')
        self.declare_parameter('tracker_3_role', 'right_elbow')

        # 获取参数
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.enable_tf = self.get_parameter('enable_tf').value
        self.enable_hmd = self.get_parameter('enable_hmd').value
        self.enable_controllers = self.get_parameter('enable_controllers').value
        self.enable_trackers = self.get_parameter('enable_trackers').value
        self.enable_body_tracking = self.get_parameter('enable_body_tracking').value
        self.num_trackers = self.get_parameter('num_trackers').value
        self.simulation_mode = self.get_parameter('simulation_mode').value

        # 构建 Tracker 角色映射
        self.tracker_roles = {
            0: self.get_parameter('tracker_0_role').value,
            1: self.get_parameter('tracker_1_role').value,
            2: self.get_parameter('tracker_2_role').value,
            3: self.get_parameter('tracker_3_role').value,
        }

        # ===== QoS 配置 =====
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ===== 发布者 =====
        # 注意: 不能使用 self.publishers，因为它是 ROS2 Node 的只读属性
        self._pubs = {}

        # 头显
        if self.enable_hmd:
            self._pubs['hmd'] = self.create_publisher(
                PoseStamped, '/pico/hmd/pose', qos)

        # 手柄
        if self.enable_controllers:
            self._pubs['controller_left'] = self.create_publisher(
                PoseStamped, '/pico/controller/left/pose', qos)
            self._pubs['controller_right'] = self.create_publisher(
                PoseStamped, '/pico/controller/right/pose', qos)
            self._pubs['controller_left_joy'] = self.create_publisher(
                Joy, '/pico/controller/left/joy', qos)
            self._pubs['controller_right_joy'] = self.create_publisher(
                Joy, '/pico/controller/right/joy', qos)

        # 追踪器 (使用语义化名称)
        if self.enable_trackers:
            for i in range(self.num_trackers):
                role = self.tracker_roles.get(i, f'tracker_{i}')
                topic_name = f'/pico/tracker/{role}'
                self._pubs[f'tracker_{i}'] = self.create_publisher(
                    PoseStamped, topic_name, qos)
                self.get_logger().info(f'Tracker #{i} → {topic_name}')

        # 全身追踪 (24关节)
        if self.enable_body_tracking:
            self._pubs['body'] = self.create_publisher(
                PoseArray, '/pico/body/poses', qos)

        # ===== TF 广播器 =====
        if self.enable_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ===== 状态 =====
        self.sdk_initialized = False
        self.lock = threading.Lock()

        # ===== 启动 SDK =====
        if HAS_XROBOTOOLKIT and not self.simulation_mode:
            try:
                # 初始化 SDK (官方 API: xrt.init())
                xrobotoolkit_sdk.init()
                self.sdk_initialized = True
                self.get_logger().info('xrobotoolkit_sdk 已初始化')
                self.get_logger().info('确保 PC-Service 正在运行: /opt/apps/roboticsservice/runService.sh')
            except Exception as e:
                self.get_logger().error(f'xrobotoolkit_sdk 初始化失败: {e}')
                self.get_logger().warn('切换到模拟模式')
                self.simulation_mode = True
        elif self.simulation_mode:
            self.get_logger().info('模拟模式已启用')
        else:
            self.get_logger().warn('xrobotoolkit_sdk 不可用，启用模拟模式')
            self.simulation_mode = True

        # ===== 定时器 =====
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_callback)

        self.get_logger().info(f'PICO Bridge 已启动 (频率: {self.publish_rate}Hz)')
        self._log_config()

    def _log_config(self):
        """打印配置信息"""
        self.get_logger().info('========== 配置信息 ==========')
        self.get_logger().info(f'  Frame ID: {self.frame_id}')
        self.get_logger().info(f'  TF 广播: {self.enable_tf}')
        self.get_logger().info(f'  头显: {self.enable_hmd}')
        self.get_logger().info(f'  手柄: {self.enable_controllers}')
        self.get_logger().info(f'  追踪器: {self.enable_trackers}')
        self.get_logger().info(f'  全身追踪: {self.enable_body_tracking}')
        if self.enable_trackers:
            self.get_logger().info('  Tracker 角色映射:')
            for i, role in self.tracker_roles.items():
                self.get_logger().info(f'    #{i} → {role}')
        self.get_logger().info(f'  模拟模式: {self.simulation_mode}')
        self.get_logger().info('================================')

    def _publish_callback(self):
        """定时发布回调"""
        now = self.get_clock().now()
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.frame_id

        if self.simulation_mode:
            self._publish_simulation_data(header)
        else:
            self._publish_real_data(header)

    def _publish_real_data(self, header: Header):
        """
        发布真实 PICO 数据

        使用 xrobotoolkit_sdk 官方 API (参考 examples/):
        - get_headset_pose() -> [x, y, z, qx, qy, qz, qw]
        - get_left_controller_pose() -> [x, y, z, qx, qy, qz, qw]
        - get_right_controller_pose() -> [x, y, z, qx, qy, qz, qw]
        - num_motion_data_available() -> int (可用 Motion Tracker 数量)
        - get_motion_tracker_pose() -> 各追踪器位姿数组
        - is_body_data_available() -> bool
        - get_body_joints_pose() -> 24x7 数组 (24 关节，每个 7 个值)
        """
        if not self.sdk_initialized:
            return

        try:
            # ===== 头显 =====
            if self.enable_hmd:
                hmd_pose = xrobotoolkit_sdk.get_headset_pose()
                if hmd_pose is not None and len(hmd_pose) >= 7:
                    pose_msg = self._create_pose_from_array(header, 'hmd', hmd_pose)
                    self._pubs['hmd'].publish(pose_msg)
                    if self.enable_tf:
                        self._broadcast_tf(header, 'hmd', pose_msg.pose)

            # ===== 手柄 =====
            if self.enable_controllers:
                # 左手柄
                left_pose = xrobotoolkit_sdk.get_left_controller_pose()
                if left_pose is not None and len(left_pose) >= 7:
                    pose_msg = self._create_pose_from_array(header, 'controller_left', left_pose)
                    self._pubs['controller_left'].publish(pose_msg)

                    # 按键状态
                    joy_msg = self._create_controller_joy(header, 'left')
                    self._pubs['controller_left_joy'].publish(joy_msg)

                    if self.enable_tf:
                        self._broadcast_tf(header, 'controller_left', pose_msg.pose)

                # 右手柄
                right_pose = xrobotoolkit_sdk.get_right_controller_pose()
                if right_pose is not None and len(right_pose) >= 7:
                    pose_msg = self._create_pose_from_array(header, 'controller_right', right_pose)
                    self._pubs['controller_right'].publish(pose_msg)

                    # 按键状态
                    joy_msg = self._create_controller_joy(header, 'right')
                    self._pubs['controller_right_joy'].publish(joy_msg)

                    if self.enable_tf:
                        self._broadcast_tf(header, 'controller_right', pose_msg.pose)

            # ===== Motion Tracker (独立追踪器) =====
            # 使用 num_motion_data_available() 和 get_motion_tracker_pose()
            if self.enable_trackers:
                self._publish_motion_tracker_data(header)

            # ===== Body Tracking (24 关节全身追踪) =====
            # 需要至少 2 个 Swift 设备
            if self.enable_body_tracking:
                if xrobotoolkit_sdk.is_body_data_available():
                    body_poses = xrobotoolkit_sdk.get_body_joints_pose()
                    if body_poses is not None:
                        body_msg = self._create_body_poses(header, body_poses)
                        self._pubs['body'].publish(body_msg)

        except Exception as e:
            self.get_logger().warn(f'获取追踪数据失败: {e}')

    def _publish_motion_tracker_data(self, header: Header):
        """
        发布 Motion Tracker 数据

        官方 API (参考 examples/example_motion_tracker.py):
        - xrt.num_motion_data_available() -> int
        - xrt.get_motion_tracker_pose() -> 位姿数组
        - xrt.get_motion_tracker_velocity() -> 速度数组
        - xrt.get_motion_tracker_serial_numbers() -> 序列号列表
        """
        try:
            num_trackers = xrobotoolkit_sdk.num_motion_data_available()
            if num_trackers == 0:
                return

            # 获取所有追踪器位姿
            tracker_poses = xrobotoolkit_sdk.get_motion_tracker_pose()
            if tracker_poses is None:
                return

            # 发布每个追踪器
            for i in range(min(num_trackers, self.num_trackers)):
                role = self.tracker_roles.get(i, f'tracker_{i}')

                # tracker_poses 是 Nx7 数组，每行 [x, y, z, qx, qy, qz, qw]
                if i < len(tracker_poses):
                    pose_data = tracker_poses[i]
                    if len(pose_data) >= 7:
                        pose_msg = self._create_pose_from_array(header, role, pose_data)
                        self._pubs[f'tracker_{i}'].publish(pose_msg)

                        if self.enable_tf:
                            self._broadcast_tf(header, role, pose_msg.pose)

        except Exception as e:
            self.get_logger().debug(f'Motion Tracker 数据获取失败: {e}')

    def _create_controller_joy(self, header: Header, side: str) -> Joy:
        """
        创建手柄 Joy 消息

        使用 xrobotoolkit_sdk 官方 API (参考 examples/example.py):
        - xrt.get_left_trigger() / get_right_trigger() -> float [0, 1]
        - xrt.get_left_grip() / get_right_grip() -> float [0, 1]
        - xrt.get_A_button() / get_B_button() -> bool (右手柄)
        - xrt.get_X_button() / get_Y_button() -> bool (左手柄)
        - xrt.get_left_axis() / get_right_axis() -> [x, y]
        """
        joy = Joy()
        joy.header = header

        try:
            if side == 'left':
                trigger = xrobotoolkit_sdk.get_left_trigger()
                grip = xrobotoolkit_sdk.get_left_grip()
                axis = xrobotoolkit_sdk.get_left_axis()
                # 左手柄: X, Y 按键
                button_primary = xrobotoolkit_sdk.get_X_button()
                button_secondary = xrobotoolkit_sdk.get_Y_button()
            else:
                trigger = xrobotoolkit_sdk.get_right_trigger()
                grip = xrobotoolkit_sdk.get_right_grip()
                axis = xrobotoolkit_sdk.get_right_axis()
                # 右手柄: A, B 按键
                button_primary = xrobotoolkit_sdk.get_A_button()
                button_secondary = xrobotoolkit_sdk.get_B_button()

            # axes: [axis_x, axis_y, trigger, grip]
            axis_x = axis[0] if axis and len(axis) > 0 else 0.0
            axis_y = axis[1] if axis and len(axis) > 1 else 0.0
            joy.axes = [
                float(axis_x),
                float(axis_y),
                float(trigger) if trigger else 0.0,
                float(grip) if grip else 0.0,
            ]

            # buttons: [primary (A/X), secondary (B/Y), menu, thumbstick_click, trigger_click, grip_click]
            joy.buttons = [
                int(button_primary) if button_primary else 0,
                int(button_secondary) if button_secondary else 0,
                0, 0, 0, 0
            ]

        except Exception:
            joy.axes = [0.0] * 4
            joy.buttons = [0] * 6

        return joy

    def _create_body_poses(self, header: Header, body_joints_pose) -> PoseArray:
        """
        创建全身 24 关节 PoseArray

        body_joints_pose: 24x7 数组
        每行格式: [x, y, z, qx, qy, qz, qw]

        关节顺序 (SMPL 24 关节):
        0: pelvis, 1: left_hip, 2: right_hip, 3: spine1,
        4: left_knee, 5: right_knee, 6: spine2, 7: left_ankle,
        8: right_ankle, 9: spine3, 10: left_foot, 11: right_foot,
        12: neck, 13: left_collar, 14: right_collar, 15: head,
        16: left_shoulder, 17: right_shoulder, 18: left_elbow, 19: right_elbow,
        20: left_wrist, 21: right_wrist, 22: left_hand, 23: right_hand
        """
        from geometry_msgs.msg import Pose
        msg = PoseArray()
        msg.header = header

        try:
            # body_joints_pose 应该是 24x7 numpy 数组
            for i in range(min(24, len(body_joints_pose))):
                pose = Pose()
                joint_data = body_joints_pose[i]

                if len(joint_data) >= 7:
                    pose.position.x = float(joint_data[0])
                    pose.position.y = float(joint_data[1])
                    pose.position.z = float(joint_data[2])
                    pose.orientation.x = float(joint_data[3])
                    pose.orientation.y = float(joint_data[4])
                    pose.orientation.z = float(joint_data[5])
                    pose.orientation.w = float(joint_data[6])
                else:
                    pose.orientation.w = 1.0

                msg.poses.append(pose)

        except Exception as e:
            self.get_logger().debug(f'Body tracking 数据解析失败: {e}')

        return msg

    def _create_pose_from_array(self, header: Header, frame_id: str, pose_array) -> PoseStamped:
        """
        从数组创建 PoseStamped

        pose_array 格式: [x, y, z, qx, qy, qz, qw]
        """
        pose = PoseStamped()
        pose.header = header
        pose.header.frame_id = frame_id

        pose.pose.position.x = float(pose_array[0])
        pose.pose.position.y = float(pose_array[1])
        pose.pose.position.z = float(pose_array[2])
        pose.pose.orientation.x = float(pose_array[3])
        pose.pose.orientation.y = float(pose_array[4])
        pose.pose.orientation.z = float(pose_array[5])
        pose.pose.orientation.w = float(pose_array[6])

        return pose

    def _broadcast_tf(self, header: Header, child_frame: str, pose):
        """广播 TF 变换"""
        t = TransformStamped()
        t.header = header
        t.child_frame_id = child_frame

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def _publish_simulation_data(self, header: Header):
        """发布模拟数据 (用于测试)"""
        t = self.get_clock().now().nanoseconds / 1e9

        # 模拟头显
        if self.enable_hmd:
            pose = PoseStamped()
            pose.header = header
            pose.header.frame_id = 'hmd'
            pose.pose.position.x = 0.0
            pose.pose.position.y = 1.6
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self._pubs['hmd'].publish(pose)
            if self.enable_tf:
                self._broadcast_tf(header, 'hmd', pose.pose)

        # 模拟手柄
        if self.enable_controllers:
            for side in ['left', 'right']:
                pose = PoseStamped()
                pose.header = header
                pose.header.frame_id = f'controller_{side}'
                pose.pose.position.x = 0.3 if side == 'right' else -0.3
                pose.pose.position.y = 1.0
                pose.pose.position.z = 0.3 + 0.05 * np.sin(t * 2)
                pose.pose.orientation.w = 1.0
                self._pubs[f'controller_{side}'].publish(pose)

                joy = Joy()
                joy.header = header
                joy.axes = [0.0] * 4
                joy.buttons = [0] * 6
                self._pubs[f'controller_{side}_joy'].publish(joy)

                if self.enable_tf:
                    self._broadcast_tf(header, f'controller_{side}', pose.pose)

        # 模拟追踪器
        if self.enable_trackers:
            for i in range(self.num_trackers):
                role = self.tracker_roles.get(i, f'tracker_{i}')
                pose = PoseStamped()
                pose.header = header
                pose.header.frame_id = role

                if role == 'left_wrist':
                    pose.pose.position.x = -0.35 + 0.02 * np.sin(t * 1.5)
                    pose.pose.position.y = 1.0
                    pose.pose.position.z = 0.35 + 0.03 * np.sin(t * 2)
                elif role == 'right_wrist':
                    pose.pose.position.x = 0.35 + 0.02 * np.sin(t * 1.5)
                    pose.pose.position.y = 1.0
                    pose.pose.position.z = 0.35 + 0.03 * np.sin(t * 2)
                elif role == 'left_elbow':
                    pose.pose.position.x = -0.25
                    pose.pose.position.y = 1.15
                    pose.pose.position.z = 0.05
                elif role == 'right_elbow':
                    pose.pose.position.x = 0.25
                    pose.pose.position.y = 1.15
                    pose.pose.position.z = 0.05
                else:
                    pose.pose.position.y = 1.0

                pose.pose.orientation.w = 1.0
                self._pubs[f'tracker_{i}'].publish(pose)

                if self.enable_tf:
                    self._broadcast_tf(header, role, pose.pose)

    def destroy_node(self):
        """清理资源"""
        # 关闭 SDK (官方 API: xrt.close())
        if self.sdk_initialized and HAS_XROBOTOOLKIT:
            try:
                xrobotoolkit_sdk.close()
                self.get_logger().info('xrobotoolkit_sdk 已关闭')
            except Exception as e:
                self.get_logger().warn(f'关闭 SDK 失败: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicoBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
