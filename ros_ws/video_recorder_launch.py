from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([        
        Node(
            package='image_view',
            name='extract',
            executable='extract_images',
            respawn="false",
            output='screen',
            cwd=LaunchConfiguration('ros_home'),
            remappings=[
                ('image', '/main_camera/image_raw'),
            ]
        )
    ])