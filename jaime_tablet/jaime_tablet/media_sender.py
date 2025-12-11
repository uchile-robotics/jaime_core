#creacion del nodo publicador de media.
#introduces la media en su formato adecuado y lo reproduce en la tablet conectada por usb (se debe activar el permiso en tablet al conectar por sub)
#es importante conectar la tablet por usb, por lo que se debe activar y conectar con adb devices por pc y activar el usb tethering y los permisos en la tablet
# al escribir adb devices en la terminal debería aparecer la tablet conectada
import rclpy
from rclpy.node import Node
import subprocess
import os

class MediaSenderNode(Node):
    def __init__(self):
        super().__init__('media_sender')

        # selección de documento (video en formato mp4, fotos en formato jpg o giff)
        #tener en cuenta que se va a abrir con google photos, por lo que se debe elegir el formato adecuado
        
        # aqui bse agrega el archivo
        #video de ejemplo (un buen video)
        self.local_path = "/home/robotica/FAUSTÃO_ Tá PEGANDO FOGO, bicho! (Domingão do Faustão).mp4"

        # Ruta destino en la tablet
        self.tablet_path = "/sdcard/Download/media_from_pc.mp4"

    def send_media(self):
        self.get_logger().info("Copiando archivo a la tablet...")

        # Enviar por ADB
        subprocess.run([
            "adb", "push", self.local_path, self.tablet_path
        ], check=True)

        self.get_logger().info("Archivo copiado. Abriendo en la tablet...")

        # Lanzar el intent para abrir el archivo
        subprocess.run([
            "adb", "shell", "am", "start",
            "-a", "android.intent.action.VIEW",
            "-d", f"file://{self.tablet_path}",
            "-t", "video/mp4"
        ], check=True)

        self.get_logger().info("Listo. Archivo enviado y abierto en la tablet.")


def main(args=None):
    rclpy.init(args=args)

    node = MediaSenderNode()
    node.send_media()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
