"""
Launch file for the group12 package
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Main function for the launch file
    """
    # find the parameter file
    parameter_file = os.path.join(
        get_package_share_directory('group12_final'),
        'config',
        'waypoint_params.yaml'
    )

    robot_controller = Node(
        package="group12_final",
        executable="robot_controller",
        parameters=[parameter_file]
    )

    ld = LaunchDescription()
    ld.add_action(robot_controller)
    return ld
