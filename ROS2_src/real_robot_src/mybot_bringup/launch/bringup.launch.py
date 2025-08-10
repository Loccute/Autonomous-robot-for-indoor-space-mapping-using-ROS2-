

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    control = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_control"),
            "launch",
            "mybot_hw_control.launch.py"
        ),
    )

    convert = Node(
        package="mybot_controller",
        executable="mybot_convert",
        name="mybot_convert"
    )


    return LaunchDescription([
        control,
        convert,
    ])
