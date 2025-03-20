from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    file_arg = DeclareLaunchArgument(
        'file', 
        default_value=PathJoinSubstitution([
            ThisLaunchFileDir(), 'data', 'img', 'video-local.avi'
        ]),
    )

    return LaunchDescription([
        file_arg,
        
        SetRemap(src='/video_file/image_raw', dst='/detection/image_in'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('video_stream_opencv'),
                    'launch',
                    'camera_launch.py'
                ])
            ]),
            launch_arguments={
                'camera_name' : 'video_file',
                'video_stream_provider' : LaunchConfiguration('file'),
                'set_camera_fps' : '30',
                'loop_videofile' : 'true',
            }.items(),
        ),
    ])