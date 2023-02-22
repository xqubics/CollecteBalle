import launch
import launch.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import os


def generate_launch_description():
    # ld = LaunchDescription()

    tennis_court_launch_file_path = os.path.join(
        os.getcwd(), 'src', 'tennis_court', 'launch', 'tennis_court.launch.py')
    robot_detector_publisher_launch_file_path = os.path.join(
        os.getcwd(), 'src', 'top_camera', 'launch', 'robot_detector_publisher.launch.py')
    robot_control_launch_file_path = os.path.join(
        os.getcwd(), 'src', 'robot_control', 'launch', 'robot_control.launch.py')

    tennis_court_launch_description = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(tennis_court_launch_file_path))
    robot_detector_publisher_launch_description = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(robot_detector_publisher_launch_file_path))
    robot_control_launch_description = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(robot_control_launch_file_path))
    # ld.add_action(robot_position_controller_node)

    return LaunchDescription([tennis_court_launch_description, robot_detector_publisher_launch_description, robot_control_launch_description])
