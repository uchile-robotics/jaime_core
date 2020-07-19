#!/usr/bin/env python

import rospy
from kobuki_msgs.msg import Sound
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import numpy as np

SOUND_ID = 0
DISTANCE_TH = 2.0
DISTANCE_TH_2 = 0.40
MIN_PERIOD = 0.25
MAX_PERIOD = 1.0

M = (MAX_PERIOD-MIN_PERIOD)/(DISTANCE_TH-DISTANCE_TH_2)
class colission_sound():

    def __init__(self):
        rospy.init_node('collision_sound')
        self.sound = True
        self.teleop = True

        self.distance_min = 4.0
        self.sound_period = 0.5

        rospy.Subscriber('/xbox_kinetic/scan',LaserScan,self.laser_callback,queue_size=1)
        rospy.Subscriber('/jaime/arduino/led/teleop',Bool,self.sound_callback,queue_size=1)
        self.sound_publisher = rospy.Publisher('/jaime/mobile_base/commands/sound', Sound, queue_size = 1)

        #Loop
        while not rospy.core.is_shutdown():
            if self.sound and self.teleop:
                if self.distance_min<DISTANCE_TH:
                    self.sound_publisher.publish(SOUND_ID)
            rospy.sleep(self.sound_period)
                    #print(self.sound_period)
            



    def sound_callback(self,msg):
        self.teleop = msg.data
        #print(self.sound)
    def laser_callback(self,msg):
        laser = msg.ranges
        laser_min = np.min(laser)
        #print(laser_min)
        self.distance_min = laser_min
        if self.distance_min < DISTANCE_TH:
            if self.distance_min > DISTANCE_TH_2:
                self.sound_period = M * self.distance_min
                self.sound = True
            else:
                self.sound_period = MIN_PERIOD
                self.sound = True
        else:
            self.sound_period = MAX_PERIOD
            self.sound = False
        return
if __name__ == '__main__':
    colission_sound()
    #try:
            
    #except:
    #    rospy.logwarn('Cannot run colission_sound')