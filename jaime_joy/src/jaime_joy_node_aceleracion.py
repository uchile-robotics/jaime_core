#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np

class Jaime_Joy():
        def __init__(self):
                self.subscriber = rospy.Subscriber('/joy', Joy, self._callback)
                self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist , queue_size=10)
                self.ctrl_speed = 0.0
                self.ctrl_turn = 0.0
                self.max_speed = 0.8
                self.max_turn = 1.0

        def _callback(self, msg):
                axes = msg.axes
                buttons = msg.buttons

                # Mapeo de Cada Boton y Ejes
                button_a = buttons[0]
                button_b = buttons[1]
                button_x = buttons[2]
                button_y = buttons[3]
                target_speed = axes[1]
                target_turn = axes[3]

                # Emergency braking
                if button_b == 1:
                        self.ctrl_speed = 0.0
                        self.ctrl_turn = 0.0
                # Suavizar velocidad para evitar jerking
                else:
                        self.ctrl_speed = self._smoother(target_speed,1)
                        self.ctrl_turn = self._smoother(target_turn, 0)

                # Crear y pub Mensaje Twist
                output = Twist()
                output.linear.x = self.ctrl_speed
                output.angular.z = self.ctrl_turn
                self.publisher.publish(output)

        def _smoother(self, trgt, angular = 0, incr = 0.02): #, self.ctrl_speed, self.ctrl_turn, self.max_spd, self.max_turn):
                """
                Recibe velocidad objetivo, anterior y maxima para hacer incrementos suaves
                y bajo el maximo permitido (podria hacerse metodo de la clase?)
                """
                if angular == 0:
                        ctrl = self.ctrl_turn
                        max_spd = self.max_turn
                else:
                        ctrl = self.ctrl_speed
                        max_spd = self.max_speed

                # velocidad incremental 
                if trgt > ctrl:
                        spd = ctrl + incr
                elif trgt == ctrl:
                        spd = trgt             
                else:
                        spd = ctrl - incr 

                # revisar si el robot se mueve para atras
                # => la parte anterior se haria con valor absoluto y la sgte con -max a max

                # Mapeo desde Joy (-1,+1) a rangos deseados de spd
                return np.interp(spd,[-1,1],[-max_spd,max_spd]) 
        


def main():
    print('iniciando Jaime Joy con aceleracion')
    print('Afirmate cabrito!!!!!!')
    rospy.init_node('Jaime_Joy')
    Jaime_Joy()
    rospy.spin()

if __name__ == '__main__':
    main()