from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  DeclareLaunchArgument




def generate_launch_description():
    
    
    node_parking_planner = Node(
        package="rototui_parking_planner",
        executable="parking_space_planner_node",
        output="screen"
    )
    
    
    
    return LaunchDescription([
        node_parking_planner,
    ])