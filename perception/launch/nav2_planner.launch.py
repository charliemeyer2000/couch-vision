"""Nav2 launch: planner_server + foxglove_bridge + optional RTAB-Map SLAM."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
    )
    nav2_params = os.path.join(config_dir, "nav2_planner_params.yaml")
    rtabmap_params = os.path.join(config_dir, "rtabmap_params.yaml")

    enable_slam_arg = DeclareLaunchArgument(
        "enable_slam",
        default_value=EnvironmentVariable("ENABLE_SLAM", default_value="0"),
    )
    enable_slam = LaunchConfiguration("enable_slam")
    slam_enabled = PythonExpression(["'", enable_slam, "' == '1'"])

    topic_prefix_arg = DeclareLaunchArgument(
        "topic_prefix",
        default_value=EnvironmentVariable("TOPIC_PREFIX", default_value="/iphone"),
    )
    topic_prefix = LaunchConfiguration("topic_prefix")

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    republish_rgb = Node(
        condition=IfCondition(slam_enabled),
        package="image_transport",
        executable="republish",
        name="republish_rgb",
        parameters=[{"in_transport": "compressed", "out_transport": "raw"}],
        remappings=[
            ("in/compressed", "/camera/image/compressed"),
            ("out", "/camera/image"),
        ],
    )

    rtabmap_slam = Node(
        condition=IfCondition(slam_enabled),
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_params],
        remappings=[
            ("rgb/image", "/camera/image"),
            ("rgb/camera_info", "/camera/camera_info"),
            ("depth/image", "/camera/depth/image"),
            ("odom", "/odom"),
        ],
    )

    static_tf_map_odom = Node(
        condition=UnlessCondition(slam_enabled),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    static_tf_odom_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_odom_base",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": 8765,
                "send_buffer_limit": 10000000,
                "num_threads": 2,
            }
        ],
    )

    return LaunchDescription(
        [
            enable_slam_arg,
            topic_prefix_arg,
            planner,
            republish_rgb,
            rtabmap_slam,
            static_tf_map_odom,
            static_tf_odom_base,
            foxglove,
        ]
    )
