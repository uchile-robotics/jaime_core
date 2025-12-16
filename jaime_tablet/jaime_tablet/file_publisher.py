import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class FilePublisherNode(Node):
    def __init__(self, file_path):
        super().__init__('file_publisher')
        
        # 🔔 Crear el publisher en el mismo tópico que el MediaSenderNode
        self.publisher_ = self.create_publisher(String, '/path', 10)
        
        # 📌 Crear un mensaje String con la ruta que deseas enviar
        msg = String()
        msg.data = file_path
        
        # 📤 Publicar el mensaje
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando la ruta: "{msg.data}" en el tópico /path')
        
        # 🛑 Detener el nodo después de publicar un solo mensaje
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Usamos un timer para apagar el nodo después de que se publique el mensaje.
        # Esto asegura que el mensaje se envíe antes de que el nodo termine.
        self.get_logger().info('Publicación completada. Terminando nodo.')
        self.timer_.cancel()
        rclpy.shutdown()

def main(args=None):
    # 📝 Verificamos que el usuario proporcione la ruta del archivo como argumento
    if len(sys.argv) < 2:
        print("Uso: ros2 run <paquete> file_publisher <ruta_del_archivo>")
        sys.exit(1)
        
    file_path = sys.argv[1] # El primer argumento después del nombre del script es la ruta
    
    rclpy.init(args=args)
    node = FilePublisherNode(file_path)
    # rclpy.spin() no es estrictamente necesario aquí si solo publicamos y terminamos,
    # pero usamos un timer para un cierre limpio.
    rclpy.spin(node) 
    # El shutdown se llama dentro del timer_callback para el cierre.

if __name__ == '__main__':
    main()