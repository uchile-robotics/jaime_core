#la idea es que inicie la camara y el envio de imagenes
#funciona iriun_node
#funciona media_sender_node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    #package jaime_tablet
    tablet_pkg = FindPackageShare('jaime_tablet')

    iriun_node = Node(
        package='jaime_tablet',
        executable='iriun_publisher',
        output='screen',
    )

    media_sender_node = Node(
        package='jaime_tablet',
        executable='media_sender',
        output='screen' 
    )

    file_publisher_node = Node(
        package='jaime_tablet',
        executable='file_publisher',
        parameters=[{
            'path': PathJoinSubstitution([
                tablet_pkg,
                'imagenes',
                'funciona.gif'
            ])
        }],
        output='screen'
    )


    return LaunchDescription([
        iriun_node,
        media_sender_node,
        file_publisher_node,
    ])
