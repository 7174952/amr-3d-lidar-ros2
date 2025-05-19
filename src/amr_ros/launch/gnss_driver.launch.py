from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取参数文件的路径
    gnss_param_file_path = os.path.join(
        get_package_share_directory('amr_ros'),
        'config',
        'zed_f9p.yaml'
    )

    ublox_gps_node = Node(package='ublox_gps',
                        executable='ublox_gps_node',
                        name='ublox_gps_node',
                        output='both',
                        remappings=[
                            ('/fix', '/fix_raw')
                        ],
                        parameters=[gnss_param_file_path]
                )
    gps_corrector_node = Node(package='amr_ros',
                        executable='gps_corrector_node',
                        name='gps_corrector_node',
                        output='both',
                        parameters=[{
                                    'gps_altitude_offset': LaunchConfiguration('gps_altitude_offset'),
                        }]
                )

    return LaunchDescription([
        DeclareLaunchArgument("gps_altitude_offset", default_value="0.8"),
        # ublox_gps_node,
        # gps_corrector_node
    ])
