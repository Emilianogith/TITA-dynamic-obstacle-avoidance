from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    gains_file = os.path.join(
        os.getenv("HOME"),
        "Desktop/ros2_ws/src/tita_controller/config/gains.yaml"
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