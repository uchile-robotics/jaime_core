from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
import os
import time
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camara_iriun',
            executable='iriun_publisher',
            name='iriun_publisher',
            output='screen'
        )
    ])

def wait_for_device(context, *args, **kwargs):
    device = '/dev/video0'
    print(f"[INFO] Esperando a que {device} aparezca...")
    while not os.path.exists(device):
        time.sleep(1)
    print(f"[INFO] {device} detectado. Iniciando publicador...")
    return [Node(
        package='camara_iriun',
        executable='iriun_publisher',
        name='iriun_publisher',
        output='screen'
    )]

def generate_launch_description():
    iriun_start = ExecuteProcess(
        cmd=['bash', '-c', 'iriunwebcam &'],
        output='screen'
    )

    return LaunchDescription([
        iriun_start,
        OpaqueFunction(function=wait_for_device),
    ])
