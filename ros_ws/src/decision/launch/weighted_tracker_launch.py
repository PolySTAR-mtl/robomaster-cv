from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_arg = DeclareLaunchArgument('params_path_camera')
    params_arg = DeclareLaunchArgument('params_path_decision')

    return LaunchDescription([
        camera_arg,
        params_arg,
        
        Node(
            package='decision',
            name='decision',
            executable='weighted_tracker',
            output='screen',
            parameters=[
                LaunchConfiguration('params_path_camera'), 
                LaunchConfiguration('params_path_decision'),
            ],
            remappings=[
                ('/decision/detections', '/detection/detections'),
                ('tracklets', 'tracking/tracklets')
            ]
        )
    ])