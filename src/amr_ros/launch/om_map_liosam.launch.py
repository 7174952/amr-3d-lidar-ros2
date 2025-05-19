from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    robot_localization_param_file_path = os.path.join(
        get_package_share_directory('amr_ros'),
        'config',
        'mapping_navsat.yaml'
    )
    gnss_ekf_param_file_path = os.path.join(
        get_package_share_directory('amr_ros'),
        'config',
        'mapping_ekf.yaml'
    )


    save_pcd_dir_arg = DeclareLaunchArgument(
        'savePCDDirectory',
        default_value='/Downloads/LOAM/',
        description='Custom directory for saving PCD maps'
    )
    gpsCovThreshold_arg = DeclareLaunchArgument(
        'gpsCovThreshold',
        default_value='0.05'
    )

    # 引用子launch文件
    livox_driver2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('livox_ros_driver2'),  # 这里替换成包含子launch文件的包名
                'launch_ROS2',
                'msg_MID360_launch.py'
            )
        )
    )

    lio_sam_launch = TimerAction(
        period=5.0,  # 延时5秒
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('lio_sam'),  # 这里替换成包含子launch文件的包名
                        'launch',
                        'run.launch.py'
                    )
                ),
                launch_arguments={
                    'savePCDDirectory': LaunchConfiguration('savePCDDirectory'),
                    'gpsCovThreshold': LaunchConfiguration('gpsCovThreshold')
                }.items()
            )
        ]
    )

    enable_gnss = LaunchConfiguration('enable_gnss')

    gnss_ekf =  TimerAction(
        period=5.0, #delay 5s
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                condition=IfCondition(enable_gnss),
                parameters=[gnss_ekf_param_file_path]
            )
        ]
    )

    navsat_location_node = TimerAction(
        period=5.0, #delay 5s
        actions=[
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform_node',
                output='screen',
                condition=IfCondition(enable_gnss),
                parameters=[robot_localization_param_file_path,
                            {
                            'datum.latitude': LaunchConfiguration('latitude'),
                            'datum.longitude': LaunchConfiguration('longitude'),
                            'datum.altitude': LaunchConfiguration('altitude'),
                            'yaw_offset': LaunchConfiguration('yaw_offset'),
                    }
                ],
                remappings=[
                    ('/gps/fix', '/fix')
                ]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_gnss',default_value='true'),
        DeclareLaunchArgument('latitude', default_value='35.0'),
        DeclareLaunchArgument('longitude', default_value='135.0'),
        DeclareLaunchArgument('altitude', default_value='0.0'),
        DeclareLaunchArgument("yaw_offset", default_value="0.0"),
        save_pcd_dir_arg,
        gpsCovThreshold_arg,
        livox_driver2_launch,
        lio_sam_launch,
        gnss_ekf,
        navsat_location_node,

    ])
