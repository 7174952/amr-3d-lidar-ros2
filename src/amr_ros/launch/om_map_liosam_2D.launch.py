from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 获取参数文件的路径
    pointcloud_to_scan_param_file_path = os.path.join(
        get_package_share_directory('amr_ros'),
        'config',
        'params_point_livox_liosam.yaml'
    )
    # 引用子launch文件
    pointcloud_to_scan_node = TimerAction(
        period=0.0,  # 延时5秒
        actions=[
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                remappings=[('cloud_in', '/lio_sam/deskew/cloud_deskewed')],
                parameters=[pointcloud_to_scan_param_file_path]
            )
        ]
    )

    slam_toolbox_launch = TimerAction(
        period=5.0,  # 延时5秒
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('amr_ros'),  # 这里替换成包含子launch文件的包名
                        'launch/include',
                        'slam_toolbox.launch.py'
                    )
                )
            )
        ]
    )

    return LaunchDescription([
        # livox_driver2_launch,
        # lio_sam_launch,
        pointcloud_to_scan_node,
        slam_toolbox_launch
    ])
