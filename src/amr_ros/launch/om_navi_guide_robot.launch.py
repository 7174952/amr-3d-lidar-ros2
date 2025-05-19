from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, IncludeLaunchDescription
import os
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('enable_gnss',default_value='true'),
        DeclareLaunchArgument('latitude', default_value='35.0'),
        DeclareLaunchArgument('longitude', default_value='135.0'),
        DeclareLaunchArgument('altitude', default_value='0.0'),
        DeclareLaunchArgument("yaw_offset", default_value="0.0"),
        DeclareLaunchArgument('gps_cov_threshold', default_value='0.5'),

        DeclareLaunchArgument('globalmap_path', default_value='/home/mikuni/ros2_ws/src/amr_ros/maps/demoRD/GlobalMap.pcd'),
        DeclareLaunchArgument('obstacle_lim', default_value='10'),
        DeclareLaunchArgument('init_px', default_value='0.0'),
        DeclareLaunchArgument('init_py', default_value='0.0'),
        DeclareLaunchArgument('init_pz', default_value='0.0'),
        DeclareLaunchArgument('init_ox', default_value='0.0'),
        DeclareLaunchArgument('init_oy', default_value='0.0'),
        DeclareLaunchArgument('init_oz', default_value='0.0'),
        DeclareLaunchArgument('init_ow', default_value='1.0'),

        DeclareLaunchArgument('v_max', default_value='0.6'),
        DeclareLaunchArgument('w_max', default_value='0.6'),
        DeclareLaunchArgument('lookahead_distance', default_value='1.0'),

        DeclareLaunchArgument('OBST_HIGHT_MIN_Z', default_value='0.0'),
        DeclareLaunchArgument('OBST_HIGHT_MAX_Z', default_value='0.8'),
        DeclareLaunchArgument('robot_width_size', default_value='0.52'),
        DeclareLaunchArgument('rvizconfig', default_value=PathJoinSubstitution(
            [get_package_share_directory('amr_ros'), 'rviz', 'guide_robot.rviz']
        )),
    ]

    # 获取参数文件的路径
    navi_param_file_path = os.path.join(
        get_package_share_directory('amr_ros'),
        'config',
        'navi_navsat.yaml'
    )
    enable_gnss = LaunchConfiguration('enable_gnss')

    # Include hdl_localization_livox.launch.py
    hdl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [get_package_share_directory('hdl_localization'), 'launch', 'hdl_localization_livox.launch.py']
        )),
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

    # Include pure_pursuit_node.launch.py
    pure_pursuit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [get_package_share_directory('pure_pursuit'), 'launch', 'pure_pursuit_node.launch.py']
        )),
        launch_arguments={
            'v_max': LaunchConfiguration('v_max'),
            'w_max': LaunchConfiguration('w_max'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance')
        }.items()
    )

    # Obstacle detection node
    detect_obstacle_node = Node(
        package='amr_ros',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen',
        parameters=[{
            'OBST_HIGHT_MIN_Z': LaunchConfiguration('OBST_HIGHT_MIN_Z'),
            'OBST_HIGHT_MAX_Z': LaunchConfiguration('OBST_HIGHT_MAX_Z'),
            'obstacle_lim': LaunchConfiguration('obstacle_lim'),
            'robot_width_size': LaunchConfiguration('robot_width_size'),
        }]
    )

    # transform to local
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

    # Static transform publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=['0.23', '0', '0', '0', '0', '0', 'base_link', 'livox_frame']
    )

    # 3D LiDAR visualization launch (Assuming converted to .launch.py)
    lidar_livox_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'rviz_MID360_launch.py']
                ))
            )
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

    return LaunchDescription(
        launch_args + [
            hdl_localization_launch,
            pure_pursuit_launch,
            detect_obstacle_node,
            navsat_trans_node,
            odom_select_node,
            static_tf,
            rviz_node,
            # lidar_livox_launch
        ]
    )
