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
- PICO Motion Tracker: 最多支持全身24关节追踪

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

xrobotoolkit_sdk 官方 API:
- get_headset_pose()              : 获取头显位姿
- get_left_controller_pose()      : 获取左手柄位姿
- get_right_controller_pose()     : 获取右手柄位姿
- get_left_controller_trigger()   : 获取左手柄扳机
- get_right_controller_trigger()  : 获取右手柄扳机
- get_left_controller_grip()      : 获取左手柄握把
- get_right_controller_grip()     : 获取右手柄握把
- get_left_controller_button_a()  : 获取左手柄A键 (实际是X键)
- get_right_controller_button_a() : 获取右手柄A键
- get_left_hand_tracking_state()  : 获取左手追踪状态
- get_right_hand_tracking_state() : 获取右手追踪状态
- get_body_tracking_state()       : 获取全身追踪状态 (24关节)
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
        self.publishers = {}

        # 头显
        if self.enable_hmd:
            self.publishers['hmd'] = self.create_publisher(
                PoseStamped, '/pico/hmd/pose', qos)

        # 手柄
        if self.enable_controllers:
            self.publishers['controller_left'] = self.create_publisher(
                PoseStamped, '/pico/controller/left/pose', qos)
            self.publishers['controller_right'] = self.create_publisher(
                PoseStamped, '/pico/controller/right/pose', qos)
            self.publishers['controller_left_joy'] = self.create_publisher(
                Joy, '/pico/controller/left/joy', qos)
            self.publishers['controller_right_joy'] = self.create_publisher(
                Joy, '/pico/controller/right/joy', qos)

        # 追踪器 (使用语义化名称)
        if self.enable_trackers:
            for i in range(self.num_trackers):
                role = self.tracker_roles.get(i, f'tracker_{i}')
                topic_name = f'/pico/tracker/{role}'
                self.publishers[f'tracker_{i}'] = self.create_publisher(
                    PoseStamped, topic_name, qos)
                self.get_logger().info(f'Tracker #{i} → {topic_name}')

        # 全身追踪 (24关节)
        if self.enable_body_tracking:
            self.publishers['body'] = self.create_publisher(
                PoseArray, '/pico/body/poses', qos)

        # ===== TF 广播器 =====
        if self.enable_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ===== 状态 =====
        self.sdk_initialized = False
        self.lock = threading.Lock()

        # ===== 启动 SDK =====
        if HAS_XROBOTOOLKIT and not self.simulation_mode:
            self.sdk_initialized = True
            self.get_logger().info('xrobotoolkit_sdk 已加载')
            self.get_logger().info('确保 PC-Service 正在运行: /opt/apps/roboticsservice/runService.sh')
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

        使用 xrobotoolkit_sdk 官方 API:
        - get_headset_pose() -> [x, y, z, qx, qy, qz, qw]
        - get_left_controller_pose() -> [x, y, z, qx, qy, qz, qw]
        - get_right_controller_pose() -> [x, y, z, qx, qy, qz, qw]
        - get_body_tracking_state() -> 24关节数据
        """
        if not self.sdk_initialized:
            return

        try:
            # ===== 头显 =====
            if self.enable_hmd:
                hmd_pose = xrobotoolkit_sdk.get_headset_pose()
                if hmd_pose is not None and len(hmd_pose) >= 7:
                    pose_msg = self._create_pose_from_array(header, 'hmd', hmd_pose)
                    self.publishers['hmd'].publish(pose_msg)
                    if self.enable_tf:
                        self._broadcast_tf(header, 'hmd', pose_msg.pose)

            # ===== 手柄 =====
            if self.enable_controllers:
                # 左手柄
                left_pose = xrobotoolkit_sdk.get_left_controller_pose()
                if left_pose is not None and len(left_pose) >= 7:
                    pose_msg = self._create_pose_from_array(header, 'controller_left', left_pose)
                    self.publishers['controller_left'].publish(pose_msg)

                    # 按键状态
                    joy_msg = self._create_controller_joy(header, 'left')
                    self.publishers['controller_left_joy'].publish(joy_msg)

                    if self.enable_tf:
                        self._broadcast_tf(header, 'controller_left', pose_msg.pose)

                # 右手柄
                right_pose = xrobotoolkit_sdk.get_right_controller_pose()
                if right_pose is not None and len(right_pose) >= 7:
                    pose_msg = self._create_pose_from_array(header, 'controller_right', right_pose)
                    self.publishers['controller_right'].publish(pose_msg)

                    # 按键状态
                    joy_msg = self._create_controller_joy(header, 'right')
                    self.publishers['controller_right_joy'].publish(joy_msg)

                    if self.enable_tf:
                        self._broadcast_tf(header, 'controller_right', pose_msg.pose)

            # ===== Motion Tracker (从全身追踪提取) =====
            if self.enable_trackers or self.enable_body_tracking:
                body_state = xrobotoolkit_sdk.get_body_tracking_state()
                if body_state is not None:
                    # 发布全身追踪
                    if self.enable_body_tracking:
                        body_msg = self._create_body_poses(header, body_state)
                        self.publishers['body'].publish(body_msg)

                    # 提取特定关节作为 Tracker
                    if self.enable_trackers:
                        self._publish_tracker_from_body(header, body_state)

        except Exception as e:
            self.get_logger().warn(f'获取追踪数据失败: {e}')

    def _create_controller_joy(self, header: Header, side: str) -> Joy:
        """
        创建手柄 Joy 消息

        使用 xrobotoolkit_sdk API:
        - get_left/right_controller_trigger() -> float [0, 1]
        - get_left/right_controller_grip() -> float [0, 1]
        - get_left/right_controller_button_a() -> bool
        - get_left/right_controller_axis_x/y() -> float [-1, 1]
        """
        joy = Joy()
        joy.header = header

        try:
            if side == 'left':
                trigger = xrobotoolkit_sdk.get_left_controller_trigger()
                grip = xrobotoolkit_sdk.get_left_controller_grip()
                button_a = xrobotoolkit_sdk.get_left_controller_button_a()
                axis_x = xrobotoolkit_sdk.get_left_controller_axis_x()
                axis_y = xrobotoolkit_sdk.get_left_controller_axis_y()
            else:
                trigger = xrobotoolkit_sdk.get_right_controller_trigger()
                grip = xrobotoolkit_sdk.get_right_controller_grip()
                button_a = xrobotoolkit_sdk.get_right_controller_button_a()
                axis_x = xrobotoolkit_sdk.get_right_controller_axis_x()
                axis_y = xrobotoolkit_sdk.get_right_controller_axis_y()

            # axes: [axis_x, axis_y, trigger, grip]
            joy.axes = [
                float(axis_x) if axis_x else 0.0,
                float(axis_y) if axis_y else 0.0,
                float(trigger) if trigger else 0.0,
                float(grip) if grip else 0.0,
            ]

            # buttons: [a/x, b/y, menu, thumbstick_click, trigger_click, grip_click]
            joy.buttons = [
                int(button_a) if button_a else 0,
                0, 0, 0, 0, 0
            ]

        except Exception:
            joy.axes = [0.0] * 4
            joy.buttons = [0] * 6

        return joy

    def _create_body_poses(self, header: Header, body_state) -> PoseArray:
        """创建全身 24 关节 PoseArray"""
        msg = PoseArray()
        msg.header = header

        # body_state 包含 24 关节的位姿、速度、加速度
        # 格式需要根据实际 SDK 返回值调整
        for i, joint_name in enumerate(self.BODY_JOINTS):
            pose = self._extract_joint_pose(body_state, i)
            msg.poses.append(pose)

        return msg

    def _extract_joint_pose(self, body_state, joint_index: int):
        """从全身追踪状态提取单个关节位姿"""
        from geometry_msgs.msg import Pose
        pose = Pose()

        try:
            # 假设 body_state 是包含所有关节数据的数组/结构
            # 实际格式需要根据 SDK 文档调整
            if hasattr(body_state, 'poses') and len(body_state.poses) > joint_index:
                joint_data = body_state.poses[joint_index]
                pose.position.x = float(joint_data[0])
                pose.position.y = float(joint_data[1])
                pose.position.z = float(joint_data[2])
                pose.orientation.x = float(joint_data[3])
                pose.orientation.y = float(joint_data[4])
                pose.orientation.z = float(joint_data[5])
                pose.orientation.w = float(joint_data[6])
            else:
                pose.orientation.w = 1.0
        except Exception:
            pose.orientation.w = 1.0

        return pose

    def _publish_tracker_from_body(self, header: Header, body_state):
        """从全身追踪数据提取 Tracker 位姿"""
        for i in range(self.num_trackers):
            role = self.tracker_roles.get(i)
            if role and role in self.TRACKER_TO_JOINT:
                joint_index = self.TRACKER_TO_JOINT[role]
                pose = self._extract_joint_pose(body_state, joint_index)

                pose_msg = PoseStamped()
                pose_msg.header = header
                pose_msg.header.frame_id = role
                pose_msg.pose = pose

                self.publishers[f'tracker_{i}'].publish(pose_msg)

                if self.enable_tf:
                    self._broadcast_tf(header, role, pose)

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
            self.publishers['hmd'].publish(pose)
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
                self.publishers[f'controller_{side}'].publish(pose)

                joy = Joy()
                joy.header = header
                joy.axes = [0.0] * 4
                joy.buttons = [0] * 6
                self.publishers[f'controller_{side}_joy'].publish(joy)

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
                self.publishers[f'tracker_{i}'].publish(pose)

                if self.enable_tf:
                    self._broadcast_tf(header, role, pose.pose)


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
