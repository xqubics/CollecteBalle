from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # joystick_node = Node(
    #     package="robot_control",
    #     executable="joystick"
    # )
    robot_position_controller_node = Node(
        package="robot_control",
        executable="robot_position_controller"
    )
    dummy_target_position_publisher_node = Node(
        package="robot_control",
        executable="dummy"
    )
    # ld.add_action(joystick_node)
    ld.add_action(robot_position_controller_node)
    # ld.add_action(dummy_target_position_publisher_node)
    return ld
