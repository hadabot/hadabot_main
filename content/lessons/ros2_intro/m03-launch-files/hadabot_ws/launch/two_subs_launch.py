from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='m03_pkg_py',
            # namespace='hadabot01',
            executable='m03_subscriber_py',
            # name='sim'
        ),
        Node(
            package='m03_pkg_cpp',
            # namespace='turtlesim2',
            executable='m03_subscriber_cpp',
            # name='sim'
        ),
        # Node(
        #    package='turtlesim',
        #    executable='mimic',
        #    name='mimic',
        #    remappings=[
        #        ('/input/pose', '/turtlesim1/turtle1/pose'),
        #        ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        #    ]
        # )
    ])
