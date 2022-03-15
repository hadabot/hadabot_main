from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='hadabot_lesson',
            executable='test_lab02_turtle_driver',
            name='turtle_driver'
        ),
        Node(
            package='hadabot_lesson',
            executable='lab02_diff_drive',
            name='diff_drive'
        )
    ])
