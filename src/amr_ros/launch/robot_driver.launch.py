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
    # 获取子launch文件的路径
    om_driver_launch_path = os.path.join(
        get_package_share_directory("om_modbus_master"),  # 这里替换成包含子launch文件的包名
        "launch",
        "om_modbus_master_launch.py"
    )

    # 引用子launch文件
    om_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(om_driver_launch_path)
    )

    return LaunchDescription([
        DeclareLaunchArgument("gear_ratio_above_20_1", default_value="False"),

        om_driver_launch,
        Node(
            package='om_cart',
            executable='om_cart_node',
            name='om_cart_node',
            output='screen',
            parameters=[{
                        "gear_ratio_above_20_1": LaunchConfiguration("gear_ratio_above_20_1"),
                    }]

        ),
    ])
