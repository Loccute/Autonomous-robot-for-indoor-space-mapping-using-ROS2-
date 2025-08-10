
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

    use_navigation = LaunchConfiguration("use_navigation")
    # use_navslam = LaunchConfiguration("use_navslam")
    # use_param = LaunchConfiguration("params_file")



    use_navigation_arg = DeclareLaunchArgument(
        "use_navigation",
        default_value="true"
    )

    # use_navslam_arg = DeclareLaunchArgument(
    #     "use_navslam",
    #     default_value="false"
    # )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_navigation"),
            "launch",
            "navigation2.launch.py"
        ),
        condition=IfCondition(use_navigation)
    )

    return LaunchDescription([
        use_navigation_arg,
        navigation,
    ])