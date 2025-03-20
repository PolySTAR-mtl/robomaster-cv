from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter, SetParametersFromFile
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument('robot_model'),
    robot_dimensions_arg = DeclareLaunchArgument('robot_model'),

    return LaunchDescription([
        robot_model_arg,
        robot_dimensions_arg,

        SetParameter(name='robot_description', value=LaunchConfiguration('robot_model')),
        SetParametersFromFile(name='robot_dimensions', file=LaunchConfiguration('robot_dimensions')),
        
        Node(
            package='tf',
            name='map',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world_frame', 'map', '10'],
        ),

        Node(
            package='tf',
            name='dead_reckoning',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom', '10'],
            remappings=[('/locate/odom', '/odom')],
        ),

        Node(
            package='tf',
            name='odom',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link', '10'],
            remappings=[('/locate/odom', '/odom')],
        ),

        Node(
            package='tf',
            name='turret',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', '1', 'base_link', 'turret', '10'],
        ),
    ])