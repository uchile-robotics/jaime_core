#creacion del nodo publicador de media.
#introduces la media en su formato adecuado y lo reproduce en la tablet conectada por usb (se debe activar el permiso en tablet al conectar por sub)
#es importante conectar la tablet por usb, por lo que se debe activar y conectar con adb devices por pc y activar el usb tethering y los permisos en la tablet
# al escribir adb devices en la terminal debería aparecer la tablet conectada


import rclpy
from rclpy.node import Node
import subprocess
import os
import time
from std_msgs.msg import String

ADB = "/usr/bin/adb"  # esto se obtiene escribiendo which adb en la terminal

class MediaSenderNode(Node):
    def __init__(self):
        super().__init__('media_sender')

        # archivo por reproducir
        self.local_path = "/home/robotica/Pato.gif"
        # Ejemplo:
        # "/home/robotica/video.mp4"
        # "/home/robotica/imagen.jpg"

        if not os.path.isfile(self.local_path):
            raise RuntimeError(f"No existe el archivo: {self.local_path}")

        filename = os.path.basename(self.local_path)
        self.tablet_path = f"/sdcard/Download/{filename}"
        
        self.subscription = self.create_subscription(
            String,
            '/path',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.local_path = msg.data
        print(self.local_path)
        self.send_media() #reescribe

    def send_media(self):
        self.get_logger().info("Enviando archivo a la tablet...")

        # Copiar archivo a la tablet
        subprocess.run(
            [ADB, "push", self.local_path, self.tablet_path],
            check=True
        )

        # 🔑 FORZAR INDEXACIÓN (para evitar problemas con apps)
        subprocess.run(
            [ADB, "shell", "am", "broadcast",
             "-a", "android.intent.action.MEDIA_SCANNER_SCAN_FILE",
             "-d", f"file://{self.tablet_path}"],
            check=True
        )

        time.sleep(1)

        self.get_logger().info("Abriendo archivo...")

        self.open_with_system_viewer()

    def open_with_system_viewer(self):
        ext = self.local_path.lower()

        # Decidir el tipo de archivo y su mime type
        if ext.endswith(".gif"):
            mime = "image/gif"
        elif ext.endswith((".jpg", ".jpeg", ".png")):
            mime = "image/*"
        elif ext.endswith((".mp4", ".mkv", ".avi")):
            mime = "video/*"
        else:
            mime = "*/*"  # Si el archivo no es reconocido, usar cualquier tipo

        # Usar el intent para abrir con la app adecuada en Android
        subprocess.run([
            ADB, "shell", "am", "start",
            "-a", "android.intent.action.VIEW",
            "-d", f"file://{self.tablet_path}",
            "-t", mime
        ], check=True)

def main(args=None):
    rclpy.init(args=args)

    node = MediaSenderNode()
    try:
        # 🌟 Esto es lo que faltaba 🌟
        # rclpy.spin() mantiene el nodo en ejecución, procesando callbacks 
        # (como self.listener_callback) de manera constante.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Permite detener el nodo con Ctrl+C
        pass
    finally:
        # Limpieza y apagado
        node.destroy_node()
        rclpy.shutdown()
    

    


if __name__ == "__main__":
    main()
