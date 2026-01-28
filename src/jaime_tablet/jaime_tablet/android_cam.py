#hay que iniciar la app ip webcam en la tablet primero (start server)
#revisar que el link en la tablet sea el mismo que sale aca
import rclpy
import cv2
import subprocess
import time
import os
import numpy as np
import imutils
import requests
from rclpy.node import Node
from sensor_msgs.msg import Image


class AndroidCam(Node):
    def __init__(self):
        super().__init__('android_cam')

        self.declare_parameter('url','http://192.168.1.133:8080/shot.jpg')
        self.declare_parameter('width', 1000)
        self.declare_parameter('height',1800)
        self.declare_parameter('fps', 30.0)

        self.url = self.get_parameter('url').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        self.timer = self.create_timer(
            1.0 / fps,
            self.timer_callback
        )

        self.get_logger().info('Fetching video from tablet...')

        
    def timer_callback(self):
        try:
            response = requests.get(self.url, timeout=1.0)
            response.raise_for_status()

            img_arr = np.frombuffer(response.content, dtype=np.uint8)
            img = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

            if img is None:
                self.get_logger().warn('Failed to decode image')
                return
            

            img = imutils.resize(img, width=self.width, height=self.height)
            cv2.imshow('android_cam', img)

            if cv2.waitKey(1) == 27:
                self.get_logger().info('ESC pressed, shutting down')
                rclpy.shutdown()
            
        except requests.RequestException as e:
            self.get_logger().warn(f'HTTP error: {e}')


    
def main():
    rclpy.init()
    node = AndroidCam()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()