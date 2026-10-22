import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_package = get_package_share_directory("my_diffbot_gazebo")
    description_package = get_package_share_directory("my_diffbot_description")

    start_imu_monitor = LaunchConfiguration("start_imu_monitor")
    start_yolo = LaunchConfiguration("start_yolo")
    start_obstacle_avoidance = LaunchConfiguration(
        "start_obstacle_avoidance"
    )
    start_rviz = LaunchConfiguration("start_rviz")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_package, "launch", "gazebo.launch.py")
        )
    )

    imu_monitor = Node(
        package="my_diffbot_navigation",
        executable="imu_monitor",
        name="imu_monitor",
        output="screen",
        condition=IfCondition(start_imu_monitor),
    )

    yolo_detector = Node(
        package="my_diffbot_vision",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen",
        condition=IfCondition(start_yolo),
    )

    obstacle_avoidance = Node(
        package="my_diffbot_navigation",
        executable="obstacle_avoidance",
        name="obstacle_avoidance",
        output="screen",
        condition=IfCondition(start_obstacle_avoidance),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(description_package, "rviz", "diffbot.rviz"),
        ],
        output="screen",
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_imu_monitor",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "start_yolo",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "start_obstacle_avoidance",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
        ),
        gazebo_launch,
        imu_monitor,
        yolo_detector,
        obstacle_avoidance,
        rviz,
    ])