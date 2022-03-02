from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='hadabot_lesson',
            executable='lab01_tf2_broadcaster',
            name='tf2_broadcaster'
        ),
        Node(
            package='hadabot_lesson',
            executable='lab01_range_sensor_driver',
            name='range_sensor_driver'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', 'launch/rviz/lab01_range_sensor_data.rviz'],
            output='screen')
    ])
