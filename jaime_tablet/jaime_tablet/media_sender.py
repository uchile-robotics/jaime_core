#creacion del nodo publicador de media.
# reproduce en la tablet conectada por usb (se debe activar el permiso en tablet al conectar por sub)
#es importante conectar la tablet por usb, por lo que se debe activar y conectar con adb devices por pc y activar el usb tethering y los permisos en la tablet
# al escribir adb devices en la terminal debería aparecer la tablet conectada
#este archivo se usa en conjunto con file_publisher. el file publisher envia el archivo a este script y así se reproduce
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import time

class MediaSenderNode(Node):
    def __init__(self):
        super().__init__('media_sender')
        self.subscription = self.create_subscription(String, '/path', self.listener_callback, 10)
        # Usamos un nombre genérico pero mantenemos la extensión original
        self.tablet_folder = "/sdcard/Download/"
        self.get_logger().info(' Receptor Inteligente Activo.')

    def listener_callback(self, msg):
        local_path = msg.data
        
        if not os.path.exists(local_path):
            self.get_logger().error(f' Archivo no encontrado en el PC: {local_path}')
            return

        # 1. Detectar tipo de archivo y preparar ruta en tablet
        extension = os.path.splitext(local_path)[1].lower() # .gif, .mp4, etc.
        mime_type = "image/gif" if extension == ".gif" else "video/mp4"
        if extension not in ['.gif', '.mp4', '.jpg', '.png']:
            mime_type = "*/*" # Intenta abrir con cualquier cosa si no es común

        remote_name = f"temp_media{extension}"
        full_tablet_path = os.path.join(self.tablet_folder, remote_name)

        try:
            # 2. Transferencia con verificación
            self.get_logger().info(f'Enviando {extension} a la tablet...')
            subprocess.run(['adb', 'push', local_path, full_tablet_path], check=True, capture_output=True)

            # 3. Pequeña pausa para que el hardware asimile el archivo
            time.sleep(0.2)

            # 4. Lanzamiento con detección de tipo
            # Usamos "action.VIEW" que es el estándar para abrir archivos
            self.get_logger().info(f'Abriendo como {mime_type}...')
            subprocess.run([
                'adb', 'shell', 'am', 'start',
                '-a', 'android.intent.action.VIEW',
                '-d', f'file://{full_tablet_path}',
                '-t', mime_type,
                '--activity-clear-top',
                '--activity-reorder-to-front'
            ], check=True, capture_output=True)

            self.get_logger().info(' Cambio completado.')

        except subprocess.CalledProcessError as e:
            error_msg = e.stderr.decode()
            self.get_logger().error(f' Error de ADB: {error_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = MediaSenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()