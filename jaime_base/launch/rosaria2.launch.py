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

    # Create and return the Node
    return LaunchDescription([
        Node(
            namespace='',
            package='kobuki',
            executable='kobuki_node',
            name='kobuki',
            parameters=[parameters],
        )
    ])
