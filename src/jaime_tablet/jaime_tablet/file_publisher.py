import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class FilePublisherNode(Node):
    def __init__(self):
        super().__init__('file_publisher')

        self.declare_parameter('path', '')
        file_path = self.get_parameter('path').get_parameter_value().string_value

        if not file_path:
            self.get_logger().error('Parámetro "path" no especificado.')
            raise RuntimeError('Missing path parameter')

        self.full_path = os.path.abspath(file_path)

        if not os.path.exists(self.full_path):
            self.get_logger().error(f'Archivo no existe: {self.full_path}')
            raise RuntimeError('File not found')

        self.publisher_ = self.create_publisher(String, '/path', 10)

        self.get_logger().info(f'Publicando ruta: {self.full_path}')
        self.timer_ = self.create_timer(0.5, self.publish_callback)

    def publish_callback(self):
        if self.publisher_.get_subscription_count() > 0:
            msg = String()
            msg.data = self.full_path
            self.publisher_.publish(msg)
            self.get_logger().info('Ruta enviada correctamente.')
            raise SystemExit
        else:
            self.get_logger().info('Esperando al nodo receptor...')

def main(args=None):
    rclpy.init(args=args)
    node = FilePublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
