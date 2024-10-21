from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("com", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("topicID", default_value="0"),
            DeclareLaunchArgument("baudrate", default_value="9600"),
            DeclareLaunchArgument("updateRate", default_value="100"),
            DeclareLaunchArgument("firstGen", default_value=""),
            DeclareLaunchArgument("secondGen", default_value=""),
            DeclareLaunchArgument("globalID", default_value="-1"),
            DeclareLaunchArgument("axisNum", default_value="1"),
            Node(
                package="om_modbus_master",
                executable="om_modbusRTU_node",
                output="screen",
                parameters=[
                    {
                        "init_com": ParameterValue(
                            LaunchConfiguration("com"), value_type=str
                        ),
                        "init_topicID": ParameterValue(
                            LaunchConfiguration("topicID"), value_type=int
                        ),
                        "init_baudrate": ParameterValue(
                            LaunchConfiguration("baudrate"), value_type=int
                        ),
                        "init_update_rate": ParameterValue(
                            LaunchConfiguration("updateRate"), value_type=int
                        ),
                        "first_gen": ParameterValue(
                            LaunchConfiguration("firstGen"), value_type=str
                        ),
                        "second_gen": ParameterValue(
                            LaunchConfiguration("secondGen"), value_type=str
                        ),
                        "global_id": ParameterValue(
                            LaunchConfiguration("globalID"), value_type=int
                        ),
                        "axis_num": ParameterValue(
                            LaunchConfiguration("axisNum"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
