#############################################################################
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node

# from launch_ros.parameters import declare_parameter
cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../rviz'
rviz_config_path = os.path.join(cur_config_path, 'hdl_localization_ros2.rviz')

def generate_launch_description():
    # arguments
    points_topic = LaunchConfiguration('points_topic', default='/livox/lidar')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id', default='base_link')
   
    init_px = LaunchConfiguration('init_px', default='0.0')
    init_py = LaunchConfiguration('init_py', default='0.0')
    init_pz = LaunchConfiguration('init_pz', default='0.0')
    init_ox = LaunchConfiguration('init_ox', default='0.0')
    init_oy = LaunchConfiguration('init_oy', default='0.0')
    init_oz = LaunchConfiguration('init_oz', default='0.0')
    init_ow = LaunchConfiguration('init_ow', default='1.0')

    # optional arguments
    use_imu                          = LaunchConfiguration('use_imu', default='true')
    invert_imu_acc                   = LaunchConfiguration('invert_imu_acc', default='false')
    invert_imu_gyro                  = LaunchConfiguration('invert_imu_gyro', default='false')
    use_global_localization          = LaunchConfiguration('use_global_localization', default='false')
    imu_topic                        = LaunchConfiguration('imu_topic', default='/livox/imu')
    enable_robot_odometry_prediction = LaunchConfiguration('enable_robot_odometry_prediction', default='true')
    robot_odom_frame_id              = LaunchConfiguration('robot_odom_frame_id', default='base_link')

    # declare node
    tf2_ros_base_link_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf2_ros_base_link_to_livox_frame',
        arguments='0.1 0.0 0.0 0.0 0.0 0.0 1.0 base_link livox_frame'.split(' '),
    )

    livox_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path]
    )

    container = ComposableNodeContainer(
        # declare_parameter('globalmap_pcd', '/home/ros2_2/src/hdl_localization/data/map.pcd'),
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::GlobalmapServer',
                name='GlobalmapServer',
                parameters=[
                    {'globalmap_pcd': '/home/mikuni/ros2_ws/src/amr_ros/maps/cloudGlobal.pcd'},
                    {'convert_utm_to_local': True},
                    {'downsample_resolution': 0.05}]),
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::HdlLocalizationNode',
                name='HdlLocalizationNode',
                # remapping
                remappings=[('/velodyne_points', points_topic), ('/gpsimu_driver/imu_data', imu_topic)],
                parameters=[
                    {'odom_child_frame_id': odom_child_frame_id},
                    {'use_imu': use_imu},
                    {'invert_acc': invert_imu_acc},
                    {'invert_gyro': invert_imu_gyro},
                    {'cool_time_duration': 2.0},
                    {'enable_robot_odometry_prediction': enable_robot_odometry_prediction},
                    {'robot_odom_frame_id': robot_odom_frame_id},
                    # <!-- available reg_methods: NDT_OMP, NDT_CUDA_P2D, NDT_CUDA_D2D-->
                    {'reg_method': 'NDT_OMP'},
                    {'ndt_neighbor_search_method': 'DIRECT7'},
                    {'ndt_neighbor_search_radius': 2.0},
                    {'ndt_resolution': 1.0},
                    {'downsample_resolution': 0.1},
                    {'specify_init_pose': True},
                    {'init_pos_x': init_px},
                    {'init_pos_y': init_py},
                    {'init_pos_z': init_pz},
                    {'init_ori_w': init_ow},
                    {'init_ori_x': init_ox},
                    {'init_ori_y': init_oy},
                    {'init_ori_z': init_oz},
                    {'use_global_localization': use_global_localization}])
        ],
        output='screen',   
    )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=False),
        tf2_ros_base_link_to_livox_frame,
        container,
        Node(
            package='hdl_global_localization',
            executable='hdl_global_localization_node',
            name='hdl_global_localization',
            output='screen',
        ),
        livox_rviz


    ])


