from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rototui_parking_planner",
            executable="parking_space_planner_node",
            output="screen"
        )
    ])