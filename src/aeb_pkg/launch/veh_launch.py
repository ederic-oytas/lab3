from __future__ import annotations
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # Create empty lists of actions
    args: list[DeclareLaunchArgument] = []
    nodes: list[Node] = []

    # Launch Arguments
    args.append(
        DeclareLaunchArgument(
            "target", description="Target node to run (in this package)."
        )
    )
    target = LaunchConfiguration("target")

    args.append(
        DeclareLaunchArgument("ttc_thresh", description="Time To Collision threshold")
    )
    ttc_thresh = LaunchConfiguration("ttc_thresh")

    # Nodes
    nodes.append(
        Node(
            package=["aeb_pkg"],
            executable=[target],
            name="safety_node",
            parameters=[
                {"base_frame": "base_link"},  # sim: ego_racecar/base_link
                {"laser_frame": "laser"},  # sim: ego_racecar/laser
                {"ttc_thresh": ttc_thresh},
            ],
        )
    )

    return LaunchDescription(args + nodes)
