from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description(*args, **kwargs):
    # Get launch configurations
    pkg = 'jaime_base'
    parameters = [os.path.join(
        get_package_share_directory(pkg),
        'params',
        'kobuki.yaml'
    )]

    ld = LaunchDescription()

    kobuki_cmd = Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        namespace='',
        output='screen',
        parameters=[parameters],
        remappings=[
            ('/commands/velocity', '/cmd_vel')
        ]
    )

    tf_footprint2base_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='',
        output='screen',
        remappings=[
            ('/tf', 'tf'), ('/tf_static', 'tf_static')
        ],
        arguments=[
            '0.0', '0.0', '0.001',
            '0.0', '0.0', '0.0',
            '1.0', 'base_link', 'base_footprint'
        ],
    )

    ld.add_action(kobuki_cmd)

    return ld
