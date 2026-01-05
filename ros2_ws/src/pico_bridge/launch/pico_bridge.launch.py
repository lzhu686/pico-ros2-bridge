#!/usr/bin/env python3
"""
PICO Bridge Launch File
启动 PICO 设备数据桥接节点

默认 Tracker 角色映射:
- Tracker #0 → left_wrist  (左腕，控制天机左臂末端)
- Tracker #1 → right_wrist (右腕，控制天机右臂末端)
- Tracker #2 → left_elbow  (左肘，提供 Elbow Hint)
- Tracker #3 → right_elbow (右肘，提供 Elbow Hint)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ===== 基础参数 =====
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='200.0',
        description='发布频率 (Hz), Motion Tracker 支持 200Hz'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='pico_world',
        description='TF 父坐标系 ID'
    )

    enable_tf_arg = DeclareLaunchArgument(
        'enable_tf',
        default_value='true',
        description='是否发布 TF 变换'
    )

    # ===== 设备启用参数 =====
    enable_hmd_arg = DeclareLaunchArgument(
        'enable_hmd',
        default_value='true',
        description='是否启用头显 (HMD) 数据'
    )

    enable_controllers_arg = DeclareLaunchArgument(
        'enable_controllers',
        default_value='true',
        description='是否启用手柄数据'
    )

    enable_trackers_arg = DeclareLaunchArgument(
        'enable_trackers',
        default_value='true',
        description='是否启用 Motion Tracker 数据'
    )

    num_trackers_arg = DeclareLaunchArgument(
        'num_trackers',
        default_value='4',
        description='Motion Tracker 数量 (最多 4 个)'
    )

    # ===== Tracker 角色映射参数 =====
    tracker_0_role_arg = DeclareLaunchArgument(
        'tracker_0_role',
        default_value='left_wrist',
        description='Tracker #0 角色: 左腕 (控制天机左臂末端 6DoF)'
    )

    tracker_1_role_arg = DeclareLaunchArgument(
        'tracker_1_role',
        default_value='right_wrist',
        description='Tracker #1 角色: 右腕 (控制天机右臂末端 6DoF)'
    )

    tracker_2_role_arg = DeclareLaunchArgument(
        'tracker_2_role',
        default_value='left_elbow',
        description='Tracker #2 角色: 左肘 (提供 Elbow Hint)'
    )

    tracker_3_role_arg = DeclareLaunchArgument(
        'tracker_3_role',
        default_value='right_elbow',
        description='Tracker #3 角色: 右肘 (提供 Elbow Hint)'
    )

    # ===== 其他参数 =====
    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='模拟模式 (无 PICO 设备时用于测试)'
    )

    # ===== PICO Bridge 节点 =====
    pico_bridge_node = Node(
        package='pico_bridge',
        executable='pico_bridge_node.py',
        name='pico_bridge',
        output='screen',
        parameters=[{
            # 基础参数
            'publish_rate': LaunchConfiguration('publish_rate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'enable_tf': LaunchConfiguration('enable_tf'),
            # 设备启用
            'enable_hmd': LaunchConfiguration('enable_hmd'),
            'enable_controllers': LaunchConfiguration('enable_controllers'),
            'enable_trackers': LaunchConfiguration('enable_trackers'),
            'num_trackers': LaunchConfiguration('num_trackers'),
            # Tracker 角色映射
            'tracker_0_role': LaunchConfiguration('tracker_0_role'),
            'tracker_1_role': LaunchConfiguration('tracker_1_role'),
            'tracker_2_role': LaunchConfiguration('tracker_2_role'),
            'tracker_3_role': LaunchConfiguration('tracker_3_role'),
            # 其他
            'simulation_mode': LaunchConfiguration('simulation_mode'),
        }]
    )

    return LaunchDescription([
        # 基础参数
        publish_rate_arg,
        frame_id_arg,
        enable_tf_arg,
        # 设备启用
        enable_hmd_arg,
        enable_controllers_arg,
        enable_trackers_arg,
        num_trackers_arg,
        # Tracker 角色
        tracker_0_role_arg,
        tracker_1_role_arg,
        tracker_2_role_arg,
        tracker_3_role_arg,
        # 其他
        simulation_mode_arg,
        # 节点
        pico_bridge_node,
    ])
