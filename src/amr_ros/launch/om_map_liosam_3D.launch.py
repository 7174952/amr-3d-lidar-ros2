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

    # 引用子launch文件
    livox_driver2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('livox_ros_driver2'),  # 这里替换成包含子launch文件的包名
                'launch_ROS2',
                'rviz_MID360_launch.py'
            )
        )
    )

    lio_sam_launch = TimerAction(
        period=15.0,  # 延时5秒
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('lio_sam'),  # 这里替换成包含子launch文件的包名
                        'launch',
                        'run.launch.py'
                    )
                )
            )
        ]
    )

    return LaunchDescription([
        livox_driver2_launch,
        lio_sam_launch,
        # pointcloud_to_scan_node,
        # slam_toolbox_launch
    ])
