from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter, SetParametersFromFile
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_arg = DeclareLaunchArgument('ekf_config'),
    model_arg = DeclareLaunchArgument('robot_model'),
    dimensions_arg = DeclareLaunchArgument('robot_dimensions'),

    return LaunchDescription([
        config_arg,
        model_arg,
        dimensions_arg,

        SetParameter(name='robot_description', value=LaunchConfiguration('robot_model')),
        SetParametersFromFile(name='robot_dimensions', file=LaunchConfiguration('robot_dimensions')),

        Node(
            package='locate',
            name='locate',
            executable='locate',
            output='screen',
        ),

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
            package='joint_state_publisher',
            name='joint_state_publisher',
            executable='joint_state_publisher'
        ), 

        Node(
            package='robot_localization',
            name='ekf',
            executable='ekf_localization_node',
            output='screen',
            parameters=[LaunchConfiguration('ekf_config')],
        ),  
    ])