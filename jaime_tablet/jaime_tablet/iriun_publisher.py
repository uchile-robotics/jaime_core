#abre el iriun desde el pc y desde la tablet, por lo que es importante haber conectado la tablet por usb y activar los permisos desde esta.
#iriun es capaz de funcionar por wi-fi (de hecho se va a conectar primero por este), pero necesita el usb para abrirse desde la tablet de forma automática
#la camara se puede acceder desde termianl por rqt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import time
import os

class IriunPublisher(Node):
    def __init__(self):
        super().__init__('iriun_publisher')
        
        #abrir iriun en tablet
        #se debe agregar el permiso para que funcione (va a aparecer al hacer correr el nodo)
        subprocess.run([
            "adb", "shell", "am", "start",
            "-n", "com.jacksoftw.webcam/com.iriun.webcam.IntroActivity"
        ])

        # --- Crear publicador ---
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()

        # --- Iniciar Iriun Webcam ---
        self.get_logger().info('Iniciando Iriun Webcam...')
        subprocess.Popen(['iriunwebcam'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # --- Esperar a que /dev/video0 esté disponible ---
        #irium crea su propia entrada, entonces es importante que espere hasta que se abra
        
        self.get_logger().info('Esperando a que /dev/video0 esté disponible...')
        for i in range(10):  # Espera hasta 10 segundos
            if os.path.exists('/dev/video0'):
                break
            time.sleep(1)
        else:
            self.get_logger().error('No se encontró /dev/video0. Iriun no se inició correctamente.')
            return

        # --- Abrir cámara
        
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir /dev/video0.')
            return

        self.get_logger().info('Publicando frames desde /dev/video0...')
        self.timer = self.create_timer(0.05, self.timer_callback)  # ~20 fps

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('No se pudo leer frame.')
            return

        # Convertir y publicar imagen
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Cerrando cámara...')
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IriunPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

