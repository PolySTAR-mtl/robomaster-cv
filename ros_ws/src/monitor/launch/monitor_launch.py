from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    return LaunchDescription([
        headless_arg,
        
        Node(
            package='monitor',
            name='monitor_logger',
            executable='monitor_logger',
            output='screen'
        ),

        Node(
            package='monitor',
            name='monitor_video',
            executable='monitor_video',
            output='screen',
            remappings=[
                ('/monitor_video/image_in', '/detection/image_in'), 
                ('/monitor_video/detections', '/detection/detections')
            ]
        ),

        Node(
            package='image_view',
            name='video_stream',
            executable='image_view',
            remappings=[
                ('image', '/monitor_video/image_out')
            ],
            condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('headless')])),
        )
    ])