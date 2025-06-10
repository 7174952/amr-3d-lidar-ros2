from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    # 获取参数文件的路径
    navi_param_file_path = os.path.join(
        get_package_share_directory('amr_ros'),
        'config',
        'navi_navsat.yaml'
    )
    enable_gnss = LaunchConfiguration('enable_gnss')

    # 引用子launch文件
    livox_driver2_launch = TimerAction(
        period=15.0,  # 延时5秒
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('livox_ros_driver2'),  # 这里替换成包含子launch文件的包名
                        'launch_ROS2',
                        'rviz_MID360_launch.py'
                    )
                )
            )
        ]
    )

    hdl_localization_launch = TimerAction(
        period=1.0,  # 延时5秒
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('hdl_localization'),  # 这里替换成包含子launch文件的包名
                        'launch',
                        'hdl_localization_livox.launch.py'
                    )
                ),
                launch_arguments={
                    'globalmap_path': LaunchConfiguration('globalmap_path'),
                    'init_px': LaunchConfiguration('init_px'),
                    'init_py': LaunchConfiguration('init_py'),
                    'init_pz': LaunchConfiguration('init_pz'),
                    'init_ox': LaunchConfiguration('init_ox'),
                    'init_oy': LaunchConfiguration('init_oy'),
                    'init_oz': LaunchConfiguration('init_oz'),
                    'init_ow': LaunchConfiguration('init_ow')
                }.items()
            )
        ]
    )

    navsat_trans_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        condition=IfCondition(enable_gnss),
        parameters=[navi_param_file_path,
                    {
                    'datum.latitude': LaunchConfiguration('latitude'),
                    'datum.longitude': LaunchConfiguration('longitude'),
                    'datum.altitude': LaunchConfiguration('altitude'),
                    'yaw_offset': LaunchConfiguration('yaw_offset'),
            }
        ],
        remappings=[
            ('/gps/fix', '/fix'),
            ('/odometry/filtered','/hdl_localization/odom')
        ]

    )

    odom_select_node = Node(
        package='amr_ros',
        executable='odom_selector_node',
        name='odom_selector_node',
        parameters=[
                    {
                    'gps_cov_threshold': LaunchConfiguration('gps_cov_threshold'),
                    'enable_gnss': LaunchConfiguration('enable_gnss')
                }
        ]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_gnss',default_value='false'),
        DeclareLaunchArgument('latitude', default_value='35.0'),
        DeclareLaunchArgument('longitude', default_value='135.0'),
        DeclareLaunchArgument('altitude', default_value='0.0'),
        DeclareLaunchArgument("yaw_offset", default_value="0.0"),
        DeclareLaunchArgument('globalmap_path', default_value='/home/mikuni/ros2_ws/src/amr_ros/maps/testCT/GlobalMap.pcd', description='Name of the robot'),
        DeclareLaunchArgument('init_px', default_value='0.0', description='init pos x'),
        DeclareLaunchArgument('init_py', default_value='0.0', description='init pos y'),
        DeclareLaunchArgument('init_pz', default_value='0.0', description='init pos z'),
        DeclareLaunchArgument('init_ox', default_value='0.0', description='init ori x'),
        DeclareLaunchArgument('init_oy', default_value='0.0', description='init ori y'),
        DeclareLaunchArgument('init_oz', default_value='0.0', description='init ori z'),
        DeclareLaunchArgument('init_ow', default_value='1.0', description='init ori z'),
        DeclareLaunchArgument('gps_cov_threshold', default_value='0.5'),
        DeclareLaunchArgument('rvizconfig', default_value=PathJoinSubstitution(
            [get_package_share_directory('amr_ros'), 'rviz', 'make_route.rviz']
        )),
        rviz_node,
        hdl_localization_launch,
        navsat_trans_node,
        odom_select_node,
        livox_driver2_launch,
    ])
