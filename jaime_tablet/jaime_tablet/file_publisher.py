#este script utiliza el media_sender y un archivo de tu elleción para reproducirlo en la tablet
#el formato a usar en terminal (luego de construit y bash) es ros2 run jaime_tablet file_publisher tu archivo
#ej: ros2 run jaime_tablet file_publisher /home/robotica/ganso.gif
#la dirección del archivo debe ser desde el computador (solo para asegurarse de que funcione bien )

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os
import time

class FilePublisherNode(Node):
    def __init__(self, file_path):
        super().__init__('file_publisher')
        self.publisher_ = self.create_publisher(String, '/path', 10)
        
        # Convertimos a ruta absoluta de inmediato
        self.full_path = os.path.abspath(file_path)
        
        # Esperamos a que el receptor esté listo
        self.get_logger().info(f'Verificando conexión para: {self.full_path}')
        self.timer_ = self.create_timer(0.5, self.publish_callback)

    def publish_callback(self):
        if self.publisher_.get_subscription_count() > 0:
            msg = String()
            msg.data = self.full_path
            self.publisher_.publish(msg)
            self.get_logger().info(' Ruta enviada correctamente.')
            # Cerramos el nodo después de enviar
            raise SystemExit 
        else:
            self.get_logger().warning('Esperando al nodo receptor...')

def main(args=None):
    if len(sys.argv) < 2:
        print("Uso: python3 file_publisher.py <nombre_archivo>")
        return
        
    rclpy.init(args=args)
    node = FilePublisherNode(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()