from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        Node(
            package='testing',
            name='serial_testing',
            executable='serial_testing',
            output='screen',
            remappings=[
                ('/serial_testing/target', '/serial/target'),
            ]
        )
    ])