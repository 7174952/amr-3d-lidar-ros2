#!/usr/bin/env python3
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_localization')
    ekf_config = os.path.join(pkg_share, 'params', 'ekf.yaml')
    navsat_config = os.path.join(pkg_share, 'params', 'navsat_transform.yaml')

    # 引用子launch文件
    ublox_gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ublox_gps'),  # 这里替换成包含子launch文件的包名
                'launch',
                'ublox_gps_node-launch.py'
            )
        )
    )

    return LaunchDescription([
        ublox_gps_launch,
        # 启动 navsat_transform_node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            remappings=[('/gps/fix', '/ublox_gps_node/fix')],
            output='screen',
            parameters=[navsat_config]
        ),
        # 启动 EKF 融合节点
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        )
    ])
