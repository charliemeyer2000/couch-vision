"""Nav2 launch: planner_server + foxglove_bridge + selectable SLAM backend.

SLAM backends (via SLAM_BACKEND env var):
  - none: No SLAM, static TF only
  - rtabmap: RTAB-Map visual SLAM (default)
  - cuvslam: Isaac ROS cuVSLAM + nvblox (Jetson only)
"""

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
    cuvslam_params = os.path.join(config_dir, "cuvslam_params.yaml")
    nvblox_params = os.path.join(config_dir, "nvblox_params.yaml")

    # SLAM backend selection: none, rtabmap, cuvslam
    slam_backend_arg = DeclareLaunchArgument(
        "slam_backend",
        default_value=EnvironmentVariable("SLAM_BACKEND", default_value="rtabmap"),
    )
    slam_backend = LaunchConfiguration("slam_backend")

    # Condition expressions
    slam_none = PythonExpression(["'", slam_backend, "' == 'none'"])
    slam_rtabmap = PythonExpression(["'", slam_backend, "' == 'rtabmap'"])
    slam_cuvslam = PythonExpression(["'", slam_backend, "' == 'cuvslam'"])
    slam_enabled = PythonExpression(["'", slam_backend, "' != 'none'"])

    topic_prefix_arg = DeclareLaunchArgument(
        "topic_prefix",
        default_value=EnvironmentVariable("TOPIC_PREFIX", default_value="/iphone"),
    )

    # --- Nav2 Planner (always runs) ---
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    # --- RTAB-Map backend ---
    republish_rgb = Node(
        condition=IfCondition(slam_rtabmap),
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
        condition=IfCondition(slam_rtabmap),
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_params],
        remappings=[
            ("rgb/image", "/camera/image"),
            ("rgb/camera_info", "/camera/camera_info"),
            ("depth/image", "/camera/depth/image"),
            ("depth/camera_info", "/camera/camera_info"),
            ("odom", "/odom"),
        ],
    )

    # --- cuVSLAM backend (Jetson only) ---
    cuvslam_node = Node(
        condition=IfCondition(slam_cuvslam),
        package="isaac_ros_visual_slam",
        executable="visual_slam_node",
        name="visual_slam",
        output="screen",
        parameters=[cuvslam_params],
        remappings=[
            ("visual_slam/image_0", "/camera/image_gray"),
            ("visual_slam/camera_info_0", "/camera/camera_info"),
            ("visual_slam/imu", "/imu"),
        ],
    )

    nvblox_node = Node(
        condition=IfCondition(slam_cuvslam),
        package="nvblox_ros",
        executable="nvblox_node",
        name="nvblox_node",
        output="screen",
        parameters=[nvblox_params],
        remappings=[
            ("depth/image", "/camera/depth/image"),
            ("depth/camera_info", "/camera/camera_info"),
            ("pose", "/visual_slam/tracking/vo_pose"),
            # Output OccupancyGrid to /map for Nav2
            ("/nvblox_node/static_map_slice", "/map"),
        ],
    )

    # --- Static TF publishers ---
    # map -> odom: Only when no SLAM (SLAM publishes this dynamically)
    static_tf_map_odom = Node(
        condition=IfCondition(slam_none),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # odom -> base_link: Always (external odom from ARKit, static identity)
    static_tf_odom_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_odom_base",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    # base_link -> camera: Only when SLAM enabled
    static_tf_base_camera = Node(
        condition=IfCondition(slam_enabled),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_camera",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "camera"],
    )

    # base_link -> imu: Only when SLAM enabled
    static_tf_base_imu = Node(
        condition=IfCondition(slam_enabled),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_imu",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu"],
    )

    # --- Foxglove bridge (always runs) ---
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
            slam_backend_arg,
            topic_prefix_arg,
            planner,
            # RTAB-Map
            republish_rgb,
            rtabmap_slam,
            # cuVSLAM + nvblox
            cuvslam_node,
            nvblox_node,
            # TF
            static_tf_map_odom,
            static_tf_odom_base,
            static_tf_base_camera,
            static_tf_base_imu,
            # Foxglove
            foxglove,
        ]
    )
