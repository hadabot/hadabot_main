from launch import LaunchDescription

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription)


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='hadabot_lesson',
            executable='solution_lab01_tf2_broadcaster',
            name='tf2_broadcaster'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'launch/rviz/solution_lab01_tf2_broadcaster.rviz'],
            output='screen')
    ])
