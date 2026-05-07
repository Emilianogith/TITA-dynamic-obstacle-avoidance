from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():

    gains_file = os.path.join(
        get_package_share_directory("tita_controller"),
        "config/gains.yaml"
    )

    return LaunchDescription([
        Node(
            package="tita_controller",
            executable="controller_node",
            name="robot_controller",
            output="screen",
            parameters=[gains_file]
        )
    ])
