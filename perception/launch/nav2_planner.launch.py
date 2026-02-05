"""Nav2 launch: planner_server + foxglove_bridge + selectable SLAM backend.

SLAM backends (via SLAM_BACKEND env var):
  - none: No SLAM, static TF only
  - rtabmap: RTAB-Map visual SLAM (default)
  - cuvslam: Isaac ROS cuVSLAM + nvblox (Jetson only)

Uses OpaqueFunction to defer node creation, avoiding parse-time package errors
when a backend's packages aren't installed.
"""

import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _create_nodes(context: LaunchContext):
    """Create nodes based on SLAM_BACKEND, only including available packages."""
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
    )
    nav2_params = os.path.join(config_dir, "nav2_planner_params.yaml")
    rtabmap_params = os.path.join(config_dir, "rtabmap_params.yaml")
    cuvslam_params = os.path.join(config_dir, "cuvslam_params.yaml")
    nvblox_params = os.path.join(config_dir, "nvblox_params.yaml")

    slam_backend = os.environ.get("SLAM_BACKEND", "rtabmap")
    nodes = []

    # --- Nav2 Planner (always runs) ---
    nodes.append(
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
        )
    )

    # --- SLAM backend nodes ---
    if slam_backend == "rtabmap":
        nodes.append(
            Node(
                package="image_transport",
                executable="republish",
                name="republish_rgb",
                parameters=[{"in_transport": "compressed", "out_transport": "raw"}],
                remappings=[
                    ("in/compressed", "/camera/image/compressed"),
                    ("out", "/camera/image"),
                ],
            )
        )
        nodes.append(
            Node(
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
        )

    elif slam_backend == "cuvslam":
        # cuVSLAM expects 1-indexed topics with num_cameras: 1
        nodes.append(
            Node(
                package="isaac_ros_visual_slam",
                executable="isaac_ros_visual_slam",
                name="visual_slam",
                output="screen",
                parameters=[cuvslam_params],
                remappings=[
                    ("visual_slam/image_1", "/camera/image_gray"),
                    ("visual_slam/camera_info_1", "/camera/camera_info"),
                    ("visual_slam/imu", "/imu"),
                ],
            )
        )
        nodes.append(
            Node(
                package="nvblox_ros",
                executable="nvblox_node",
                name="nvblox_node",
                output="screen",
                parameters=[nvblox_params],
                remappings=[
                    ("depth/image", "/camera/depth/image"),
                    ("depth/camera_info", "/camera/camera_info"),
                    ("pose", "/visual_slam/tracking/vo_pose"),
                    ("/nvblox_node/static_map_slice", "/map"),
                ],
            )
        )

    # --- Static TF publishers ---
    # map -> odom: Only when no SLAM (SLAM publishes this dynamically)
    if slam_backend == "none":
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_map_odom",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            )
        )

    # odom -> base_link: Always needed
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_odom_base",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        )
    )

    # base_link -> camera/imu: Only when SLAM enabled
    if slam_backend in ("rtabmap", "cuvslam"):
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_base_camera",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "camera"],
            )
        )
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_base_imu",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu"],
            )
        )

    # --- Foxglove bridge (always runs) ---
    nodes.append(
        Node(
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
    )

    return nodes


def generate_launch_description():
    slam_backend_arg = DeclareLaunchArgument(
        "slam_backend",
        default_value=os.environ.get("SLAM_BACKEND", "rtabmap"),
        description="SLAM backend: none, rtabmap, or cuvslam",
    )

    return LaunchDescription(
        [
            slam_backend_arg,
            OpaqueFunction(function=_create_nodes),
        ]
    )
