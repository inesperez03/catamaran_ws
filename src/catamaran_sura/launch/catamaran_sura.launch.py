from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    environment = LaunchConfiguration("environment")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("catamaran_bringup"),
                "launch",
                "bringup.launch.py"
            )
        ),
        launch_arguments={"environment": environment}.items()
    )

    stonefish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("catamaran_stonefish"),
                "launch",
                "catamaran_cirtesu.launch.py"
            )
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("environment"), "' == 'sim'"])
        )
    )
    
    navigator_node = Node(
        package="catamaran_navigator",
        executable="navigator_sim", 
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("environment"), "' == 'sim'"])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "environment",
            default_value="real"
        ),

        bringup_launch,
        stonefish_launch,
        navigator_node
    ])