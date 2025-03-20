from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    params_arg = DeclareLaunchArgument('params_path')
    workspace_arg = DeclareLaunchArgument('workspace')
    darknet_arg = DeclareLaunchArgument('darknet', default_value='True')
    deepstream_arg = DeclareLaunchArgument('deepstream', default_value='False')

    return LaunchDescription([
        params_arg,
        workspace_arg,
        darknet_arg,
        deepstream_arg,
        
        Node(
            package='detection',
            name='detection',
            executable='detection',
            output='screen',
            parameters=[LaunchConfiguration('params_path')],
            condition=IfCondition(LaunchConfiguration('darknet')),
        ),

        Node(
            package='detection',
            name='detection',
            executable='deepstream_detection',
            output='screen',
            parameters=[LaunchConfiguration('params_path')],
            condition=IfCondition(LaunchConfiguration('deepstream')),
        ),
    ])