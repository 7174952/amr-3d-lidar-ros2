from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lio_sam'),  # 这里替换成包含子launch文件的包名
                'launch',
                'run.launch.py'
            )
        )
    )
    livox_driver2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('livox_ros_driver2'),  # 这里替换成包含子launch文件的包名
                'launch_ROS2',
                'msg_MID360_launch.py'
            )
        )
    )
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('amr_ros'),
                'launch/include',
                'slam_toolbox.launch.py'
            )
        ]),
    )
    return LaunchDescription([
        lio_sam_launch,
        slam_toolbox_launch,
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[('cloud_in', '/lio_sam/deskew/cloud_deskewed')],
            parameters=[pointcloud_to_scan_param_file_path]
        ),
        livox_driver2_launch

    ])
