from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                {'magnetic_declination_radians': 0.0},
                {'yaw_offset': 0.0},
                {'broadcast_cartesian_transform': False},
                {'publish_filtered_gps': False},
                {'use_odometry_yaw': False},
                {'wait_for_datum': False},
                {'frequency': 10.0},
                {'delay': 0.0},
                {'base_link_frame': 'base_link'},
                {'world_frame': 'map'},
                {'datum': [35.27314007, 139.1320541, 91.9307]}
            ],
            remappings=[
                ('/gps/fix', '/fix'),
                ('/odometry/filtered', '/odometry/gps')
            ]
        )
    ])
