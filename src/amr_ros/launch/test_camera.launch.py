from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_ros',
            executable='person_detect',
            name='realsense_yolo_node',
            output='screen'
        )
    ])
