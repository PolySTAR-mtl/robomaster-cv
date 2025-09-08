from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        Node(
            package='tf',
            name='static_turret',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', '1', 'base_link', 'turret', '10']
        ),
    ])