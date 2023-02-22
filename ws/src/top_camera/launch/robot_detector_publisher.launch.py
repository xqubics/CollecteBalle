from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    robot_detector_publisher_node = Node(
        package="top_camera",
        executable="robot_detector_publisher"
    )
    ld.add_action(robot_detector_publisher_node)
    return ld
