from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os 

def generate_launch_description():

    rviz_config_path = os.path.join(os.path.abspath(os.getcwd()),"src/image_path_planning/config","image_planner_view.rviz")

    return LaunchDescription([

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        Node(
            package='waypoint_generator',
            executable='waypoint_publisher',
            name='waypoint_publisher_node'
        ),

        Node(
            package='image_planner',
            executable='tentacle_planner',
            name='tentacle_planner_node'
        ),

        Node(
            package='controller',
            executable='control',
            name='control_node'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    ])