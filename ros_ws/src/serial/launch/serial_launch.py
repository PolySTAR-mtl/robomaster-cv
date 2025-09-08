from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    params_arg = DeclareLaunchArgument('params_path')

    return LaunchDescription([
        params_arg,
        
        Node(
            package='serial',
            name='serial',
            executable='serial_interface',
            output='screen',
            parameters=[LaunchConfiguration('params_path')],
        )
    ])