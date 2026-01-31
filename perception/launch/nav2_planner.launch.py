"""Minimal Nav2 launch: planner_server + lifecycle_manager + static TF."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
        "nav2_planner_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager",
            output="screen",
            parameters=[params_file],
        ),
        # Static TF: map → base_link (identity — ego-centric)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
        ),
    ])
