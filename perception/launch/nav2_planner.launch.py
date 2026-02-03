"""Minimal Nav2 launch: planner_server + lifecycle_manager + foxglove_bridge + static TF."""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
        "nav2_planner_params.yaml",
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file],
    )

    # Delay lifecycle_manager so planner_server has time to register its services
    lifecycle = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                output="screen",
                parameters=[params_file],
            ),
        ],
    )

    return LaunchDescription([
        planner,
        lifecycle,
        # Static TF: map → base_link (identity — ego-centric)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
        ),
        # Foxglove bridge: forwards all ROS2 topics to ws://localhost:8765
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": 8765,
                "send_buffer_limit": 10000000,
                "num_threads": 2,
            }],
        ),
    ])
