"""Launch the synthetic end-to-end detection and tracking check."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception",
                executable="detect",
                name="detect",
                parameters=[{"detect.deskew": False}],
                output="screen",
            ),
            Node(
                package="perception",
                executable="tracking",
                name="tracking",
                output="screen",
            ),
            Node(
                package="perception",
                executable="fake_perception_test",
                name="fake_perception_test",
                output="screen",
            ),
        ]
    )
