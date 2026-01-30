#la idea es que inicie la camara y el envio de imagenes
#funciona file_publisher
#funciona media_sender_node
#funciona android_cam
#se debe activar IP webcam en la tablet primero

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

    android_cam_node = Node(
        package='jaime_tablet',
        executable='android_cam',
        output='screen'
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
                'pato.gif'
            ])
        }],
        output='screen'
    )


    return LaunchDescription([
        android_cam_node,
        media_sender_node,
        file_publisher_node,
    ])
