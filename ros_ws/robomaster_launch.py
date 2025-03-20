from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, SetParametersFromFile, IncludeLaunchDescription, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir, FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    workspace_arg = DeclareLaunchArgument('workspace', default_value=ThisLaunchFileDir())
    simu_arg = DeclareLaunchArgument('simu', default_value='false')

    return LaunchDescription([
        workspace_arg,
        simu_arg,
        deepstream_arg,
        darknet_arg,

        SetRemap(src='/serial/target', dst='/decision/target'),

        SetParametersFromFile(
            file=PathJoinSubstitution([
                LaunchConfiguration('workspace'), 'data', 'models', 'icra.yaml'
            ]),
        ),

        SetParametersFromFile(
            file=PathJoinSubstitution([
                LaunchConfiguration('workspace'), 'data', 'param-camera.yaml'
            ]),
            namespace='camera',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('decision'),
                    'launch',
                    'simple_decision_launch.py'
                ])
            ]),
            launch_arguments={
                'params_path': PathJoinSubstitution([
                    LaunchConfiguration('workspace'), 'data', 'param-decision.yaml'
                ])
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('serial'),
                    'launch',
                    'serial_launch.py'
                ])
            ]),
            launch_arguments={
                'params_path': PathJoinSubstitution([
                    LaunchConfiguration('workspace'), 'data', 'param-serial.yaml'
                ])
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('locate'),
                    'launch',
                    'mock_launch.py'
                ])
            ]),
            launch_arguments={
                'robot_dimensions': PathJoinSubstitution([
                    LaunchConfiguration('workspace'), 'data', 'models', 'icra.yaml'
                ]),
                'robot_model': PathJoinSubstitution([
                    LaunchConfiguration('workspace'), 'data', 'models', 'icra.xml'
                ]),
            }.items(),
            condition=IfCondition(LaunchConfiguration('simu')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('detection'),
                    'launch',
                    'detection_launch.py'
                ])
            ]),
            launch_arguments={
                'workspace': LaunchConfiguration('workspace'),
                'params_path': PathJoinSubstitution([
                    LaunchConfiguration('workspace'), 'data', 'param-yolo.yaml'
                ]),
                'deepstream': 'true',
                'darknet': 'false',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tracking'),
                    'launch',
                    'tracking_launch.py'
                ])
            ]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('locate'),
                    'launch',
                    'locate_launch.py'
                ])
            ]),
            launch_arguments={
                'ekf_config': PathJoinSubstitution([
                    LaunchConfiguration('workspace'), 'data', 'param-localization.yaml'
                ]),
                'robot_dimensions': PathJoinSubstitution([
                    LaunchConfiguration('workspace'), 'data', 'models', 'icra.yaml'
                ]),
                'robot_model': PathJoinSubstitution([
                    LaunchConfiguration('workspace'), 'data', 'models', 'icra.xml'
                ]),
            }.items(),
            condition=UnlessCondition(LaunchConfiguration('simu')),
        ),
    ])