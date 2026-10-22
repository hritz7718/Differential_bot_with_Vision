import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory("my_diffbot_description")

    xacro_file = os.path.join(pkg_path, "urdf", "diffbot.urdf.xacro")
    rviz_file = os.path.join(pkg_path, "rviz", "diffbot.rviz")

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {
        "robot_description": robot_description_config.toxml()
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        output="screen",
    )

    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])
