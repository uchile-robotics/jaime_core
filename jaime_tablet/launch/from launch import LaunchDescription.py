from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Ejecuta Iriun Webcam
        ExecuteProcess(
            cmd=['iriunwebcam'],
            output='screen'
        ),

        # Espera 5 segundos y luego ejecuta ffplay
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 5 && ffplay /dev/video0'],
            output='screen'
        ),
    ])
