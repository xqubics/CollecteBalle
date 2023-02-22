from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    path_point_publisher_node = Node(
        package="top_camera",
        executable="path_point_publisher"
    )
    ld.add_action(path_point_publisher_node)
    return ld
