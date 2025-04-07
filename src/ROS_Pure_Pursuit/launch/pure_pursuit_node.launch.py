import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    wheel_base_arg            = DeclareLaunchArgument('wheel_base', default_value='0.5', description='wheelbase')
    lookahead_distance_arg    = DeclareLaunchArgument('lookahead_distance', default_value='1.0', description='lookahead distance')
    v_max_arg                 = DeclareLaunchArgument('v_max', default_value='0.6', description='run velocity')
    w_max_arg                 = DeclareLaunchArgument('w_max', default_value='0.8', description='max rotational velocity')
    position_tolerance_arg    = DeclareLaunchArgument('position_tolerance', default_value='0.1', description='position tolerance')
    orientation_tolerance_arg = DeclareLaunchArgument('orientation_tolerance', default_value='0.1', description='orientation tolerance')
    delta_vel_arg             = DeclareLaunchArgument('delta_vel', default_value='10.0', description='steering angle velocity')
    acc_arg                   = DeclareLaunchArgument('acc', default_value='0.1', description='acceleration')
    dec_arg                   = DeclareLaunchArgument('dec', default_value='0.1', description='deceleration')
    jerk_arg                  = DeclareLaunchArgument('jerk', default_value='1.0', description='jerk')
    delta_max_arg             = DeclareLaunchArgument('delta_max', default_value='3.14', description='steering angle limit')
    map_frame_id_arg          = DeclareLaunchArgument('map_frame_id', default_value='map', description='map frame id')
    robot_frame_id_arg        = DeclareLaunchArgument('robot_frame_id', default_value='base_link', description='robot frame id')
    lookahead_frame_id_arg    = DeclareLaunchArgument('lookahead_frame_id', default_value='lookahead', description='lookahead frame id')
    acker_frame_id_arg        = DeclareLaunchArgument('acker_frame_id', default_value='base_link', description='ackermann frame id')

    # 配置纯跟踪节点
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        parameters=[{
            'wheelbase': LaunchConfiguration('wheel_base'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'max_linear_velocity': LaunchConfiguration('v_max'),
            'max_rotational_velocity': LaunchConfiguration('w_max'),
            'position_tolerance': LaunchConfiguration('position_tolerance'),
            'orientation_tolerance': LaunchConfiguration('orientation_tolerance'),
            'steering_angle_velocity': LaunchConfiguration('delta_vel'),
            'acceleration': LaunchConfiguration('acc'),
            'deceleration': LaunchConfiguration('dec'),
            'jerk': LaunchConfiguration('jerk'),
            'steering_angle_limit': LaunchConfiguration('delta_max'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'robot_frame_id': LaunchConfiguration('robot_frame_id'),
            'lookahead_frame_id': LaunchConfiguration('lookahead_frame_id'),
            'acker_frame_id': LaunchConfiguration('acker_frame_id'),
        }]
    )

    return LaunchDescription([
        wheel_base_arg,
        lookahead_distance_arg,
        v_max_arg,
        w_max_arg,
        position_tolerance_arg,
        orientation_tolerance_arg,
        delta_vel_arg,
        acc_arg,
        dec_arg,
        jerk_arg,
        delta_max_arg,
        map_frame_id_arg,
        robot_frame_id_arg,
        lookahead_frame_id_arg,
        acker_frame_id_arg,

        pure_pursuit_node,
    ])
