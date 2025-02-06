from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_distance',
            executable='left_lidar',
            name='left_lidar_node'
        ),
        Node(
            package='lidar_distance',
            executable='right_lidar',
            name='right_lidar_node'
        )
    ])
