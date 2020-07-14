#!/usr/bin/env python

import roslib
import rospy
from kobuki_msgs.msg import Sound
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Twist
import subprocess

ERROR_SOUND = 4
CMD_ZERO = Twist()
class Shutdown_srv():


    def __init__(self):
        rospy.init_node('shutdown_srv')
        # variables  
        self.shutdown = False 
        self.speed = 0  
        self.count = 0
        # Topics
        rospy.Subscriber('base_vel', Twist, self.callbackSpeed)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.sound_pub = rospy.Publisher('sound', Sound, queue_size = 10)
        # Service
        self.s = rospy.Service('shutdown_service', Trigger, self.callbackShutdown)
        
        #Loop
        while not rospy.core.is_shutdown():
            if self.shutdown:
                # Shutdown if speed is 0 or publish zero speed for 1 second
                if (self.speed == 0 and self.count==0) or self.count == 5:
                    rospy.loginfo("shutdown")
                    subprocess.call(["systemctl","poweroff"])
                else:
                    self.vel_pub.publish(CMD_ZERO)
                    self.count += 1
            rospy.sleep(0.2)

    def callbackSpeed(self, msg):
        linear = abs(msg.linear.x)
        angular = abs(msg.angular.z)
        self.speed = linear + angular
        #rospy.loginfo(str(self.speed))

    def callbackShutdown(self, req):
        self.shutdown = True
        self.sound_pub.publish(ERROR_SOUND)
        return TriggerResponse(True, "Begin Shutdown")

if __name__ == '__main__':
    try:
        Shutdown_srv()
    except:
        rospy.logwarn('Cannot run shutdown_srv')

