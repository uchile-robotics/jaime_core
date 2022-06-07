#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Jaime_Joy():
    def __init__(self):
        self.subscriber = rospy.Subscriber('/joy', Joy, self._callback)
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist , queue_size=10)
        
    def _callback(self, msg):

        axes = msg.axes
        buttons = msg.buttons

        adelante = axes[1]
        girar = axes[3]
        
        button_a = buttons[0]
        button_b = buttons[1]
        button_x = buttons[2]
        button_y = buttons[3]

        output = Twist()
        
        output.linear.x = adelante 
        output.angular.z = girar
        
        self.publisher.publish(output)
            

def main():

    rospy.init_node('Jaime_Joy')
    Jaime_Joy()
    rospy.spin()

if __name__ == '__main__':
    main()