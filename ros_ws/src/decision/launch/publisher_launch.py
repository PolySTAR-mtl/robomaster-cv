from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='decision',
            executable='publisher', 
            name='publisher',
            output='screen',
            remappings=[
                ('tracklets', '/tracking/tracklets')
            ]
        )
    ])
