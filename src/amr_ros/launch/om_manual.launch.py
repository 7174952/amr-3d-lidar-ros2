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

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/input/js0"),

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                        "device": LaunchConfiguration("device"),
                    }]
        ),
        Node(
            package='om_cart',
            executable='om_manual_node',
            name='om_manual_node',
            output='screen',
        )
    ])
