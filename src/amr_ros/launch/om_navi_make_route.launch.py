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

    hdl_localization_launch = TimerAction(
        period=5.0,  # 延时5秒
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('hdl_localization'),  # 这里替换成包含子launch文件的包名
                        'launch',
                        'hdl_localization_livox.launch.py'
                    )
                )
            )
        ]
    )

    return LaunchDescription([
        livox_driver2_launch,
        hdl_localization_launch,
    ])
