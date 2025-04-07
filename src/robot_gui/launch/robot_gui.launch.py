from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='robot_gui',
            executable='robot_gui_node',
            name='robot_gui',
            output='screen',
        )
    ])
