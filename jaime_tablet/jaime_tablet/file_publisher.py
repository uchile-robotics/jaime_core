#este script utiliza el media_sender y un archivo de tu elleción para reproducirlo en la tablet
#el formato a usar en terminal (luego de construit y bash) es ros2 run jaime_tablet file_publisher tu archivo
#ej: ros2 run jaime_tablet file_publisher /home/robotica/ganso.gif
#la dirección del archivo debe ser desde el computador (solo para asegurarse de que funcione bien )

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import subprocess
import time

class MasterPublisher(Node):
    def __init__(self, file_path):
        super().__init__('master_publisher')
        self.file_path = file_path
        
        # 1. LANZAR EL RECEPTOR AUTOMÁTICAMENTE
        # Esto abre el 'media_sender' en segundo plano sin que tengas que abrir otra terminal
        self.get_logger().info('Iniciando el nodo receptor (media_sender) en segundo plano...')
        try:
            # Reemplaza 'mi_paquete' por el nombre real de tu paquete
            self.process = subprocess.Popen(
                ['ros2', 'run', 'jaime_tablet', 'media_sender'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
        except Exception as e:
            self.get_logger().error(f'No se pudo iniciar media_sender: {e}')
            return

        # 2. Configurar el Publicador
        self.publisher_ = self.create_publisher(String, '/path', 10)
        
        # 3. Esperar a que el sistema ROS 2 se estabilice
        self.get_logger().info('Esperando 3 segundos a que el receptor esté listo...')
        time.sleep(3.0)
        
        self.publish_and_exit()

    def publish_and_exit(self):
        msg = String()
        msg.data = self.file_path
        
        # Publicamos varias veces para asegurar que el receptor recién abierto lo capture
        for i in range(3):
            self.publisher_.publish(msg)
            self.get_logger().info(f'Enviando archivo ({i+1}/3): {msg.data}')
            time.sleep(0.5)
            
        self.get_logger().info('Proceso completado. El archivo debería estar reproduciéndose.')
        self.get_logger().info('Nota: El receptor seguirá corriendo en segundo plano.')
        
        # Terminamos el proceso del publicador
        raise SystemExit

def main(args=None):
    if len(sys.argv) < 2:
        print("Uso: python3 file_publisher.py /ruta/al/video.mp4")
        return
        
    file_path = sys.argv[1]
    
    rclpy.init(args=args)
    node = MasterPublisher(file_path)
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()