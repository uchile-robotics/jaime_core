#creacion del nodo publicador de media.
#introduces la media en su formato adecuado y lo reproduce en la tablet conectada por usb (se debe activar el permiso en tablet al conectar por sub)
#es importante conectar la tablet por usb, por lo que se debe activar y conectar con adb devices por pc y activar el usb tethering y los permisos en la tablet
# al escribir adb devices en la terminal debería aparecer la tablet conectada
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class MediaSenderNode(Node):
    def __init__(self):
        super().__init__('media_sender')
        
        # Suscripción al tópico donde el publisher envía la ruta
        self.subscription = self.create_subscription(
            String,
            '/path',
            self.listener_callback,
            10)
        
        self.get_logger().info('🚀 Nodo Receptor Listo. Esperando rutas de archivos...')
        
        # Configuración de rutas en la Tablet
        self.tablet_path = "/sdcard/Download/temp_media.gif"

    def listener_callback(self, msg):
        local_file_path = msg.data
        self.get_logger().info(f'📥 Recibido: "{local_file_path}"')

        # 1. Validar si el archivo local existe
        if not os.path.exists(local_file_path):
            self.get_logger().error(f'❌ El archivo local no existe en: {local_file_path}')
            return

        try:
            # 2. Detener cualquier visualizador previo para forzar el refresco
            # Usamos un comando genérico para cerrar aplicaciones de galería/fotos
            self.get_logger().info('Refrescando pantalla de la tablet...')
            subprocess.run(['adb', 'shell', 'am', 'force-stop', 'com.google.android.apps.photos'], check=False)
            subprocess.run(['adb', 'shell', 'am', 'force-stop', 'com.android.gallery3d'], check=False)

            # 3. Transferir el archivo vía ADB (Sobreescribiendo el anterior)
            self.get_logger().info(f'Enviando archivo a la tablet...')
            result = subprocess.run(['adb', 'push', local_file_path, self.tablet_path], capture_output=True, text=True)
            
            if result.returncode != 0:
                self.get_logger().error(f'❌ Error de ADB: {result.stderr}')
                return

            # 4. Abrir el archivo en la tablet con un Intent
            # El flag --activity-clear-task ayuda a que no se quede pegado en la imagen anterior
            self.get_logger().info('Reproduciendo en la tablet...')
            subprocess.run([
                'adb', 'shell', 'am', 'start',
                '-a', 'android.intent.action.VIEW',
                '-d', f'file://{self.tablet_path}',
                '-t', 'image/gif',
                '--activity-clear-task'
            ], check=True)

            self.get_logger().info('✅ ¡GIF actualizado con éxito!')

        except Exception as e:
            self.get_logger().error(f'🔥 Error crítico durante la ejecución: {str(e)}')

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