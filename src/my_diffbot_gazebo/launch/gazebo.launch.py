import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    desc_pkg = get_package_share_directory("my_diffbot_description")
    gazebo_pkg = get_package_share_directory("my_diffbot_gazebo")

    xacro_file = os.path.join(desc_pkg, "urdf", "diffbot.urdf.xacro")
    world_file = os.path.join(gazebo_pkg, "worlds", "obstacle_world.world")

    robot_description = xacro.process_file(xacro_file).toxml()

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={"world": world_file}.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen",
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "diffbot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_robot_node,
    ])
