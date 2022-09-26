from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_generator',
            namespace='ns1',
            executable='waypoint_publisher',
            name='waypoint_publisher_node'
        )
    ])