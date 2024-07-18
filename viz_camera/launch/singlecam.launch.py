import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    muxer_config = os.path.join(
        get_package_share_directory("viz_camera"),
        "config",
        "muxer.yaml",
    )

    viewer_config = os.path.join(
        get_package_share_directory("viz_camera"),
        "config",
        "viewer.rviz",
    )

    muxer_node = Node(
        package="viz_camera",
        executable="muxer",
        name="muxer",
        namespace="muxer",
        parameters=[muxer_config],
        remappings=[
            ("images", "/flir_camera/image_raw"),
            ("camera_info", "/flir_camera/camera_info"),
            ("detections", "/detections")
        ],
    )
    
    viewer_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", viewer_config],
    )

    return LaunchDescription(
        [
           muxer_node,
           viewer_node,
        ]
    )
    
