#!/usr/bin/env python

import rospy
from kobuki_msgs.msg import Sound
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import Twist


#TODO Dejar estos como parametros ROS
SOUND_ID = 0
DISTANCE_TH = 2.0
DISTANCE_TH_2 = 0.40
MIN_PERIOD = 0.25
MAX_PERIOD = 1.0

MIN_FACTOR = 0
DISTANCE_MAX = 1.0
DISTANCE_MIN = 0.40

THETA_RANGE_INF = -1.436332 # 30 Degree
THETA_RANGE_MAX = 1.436332


M = (MAX_PERIOD-MIN_PERIOD)/(DISTANCE_TH-DISTANCE_TH_2)
M_SAFETY = 1 / (DISTANCE_MAX-DISTANCE_MIN)

Y_RANGE = 0.2
class teleop_safety():

    def __init__(self):
        rospy.init_node('teleop_safety')
        self.sound = True

        self.distance_min = 4.0
        self.sound_period = 0.5
        self.safety_factor = 1

        rospy.Subscriber('/xbox_kinetic/scan',LaserScan,self.laser_callback,queue_size=1)
        rospy.Subscriber('/jaime/arduino/led/teleop',Bool,self.sound_callback,queue_size=1)
        rospy.Subscriber('joy_command',Twist,self.controller_callback,queue_size=10)
        
        self.cmd_pub = rospy.Publisher('safety_command',Twist,queue_size=10)
        self.sound_publisher = rospy.Publisher('/jaime/mobile_base/commands/sound', Sound, queue_size = 1)

        #Loop
        while not rospy.core.is_shutdown():
            if self.sound:
                if self.distance_min<DISTANCE_TH:
                    self.sound_publisher.publish(SOUND_ID)
                    rospy.sleep(self.sound_period)
                    #print(self.sound_period)
            


    def controller_callback(self,msg):
        if msg.linear.x > 0:
            msg.linear.x = msg.linear.x * self.safety_factor
        else:
            msg.linear.x = msg.linear.x * 0.5
        self.cmd_pub.publish(msg)
        return

    def sound_callback(self,msg):
        self.sound = msg.data
        #print(self.sound)
    
    def get_sound_period(self,laser_ranges):
        laser_min = np.min(laser_ranges)
        #print(laser_min)
        self.distance_min = laser_min
        if self.distance_min < DISTANCE_TH:
            if self.distance_min > DISTANCE_TH_2:
                self.sound_period = M * self.distance_min
            else:
                self.sound_period = MIN_PERIOD
        else:
            self.sound_period = MAX_PERIOD
        return
    
    #TODO Pendiente hacer que el robot vea si va en una curva
    def laser_callback(self,msg):
        #self.get_sound_period(msg.ranges)

        theta_i = msg.angle_min
        min_range = 5.0
        laser_distance = 0


        for r in msg.ranges:
            if THETA_RANGE_INF < theta_i < THETA_RANGE_MAX:
                dx_ = r * np.cos(theta_i)
                dy = r * np.sin(theta_i)
                dx = laser_distance + dx_
                if abs(dy)>Y_RANGE:
                    pass
                elif dx < min_range:
                    min_range=dx
            theta_i += msg.angle_increment 
        #rospy.loginfo("min range {}".format(min_range))
        
        if min_range < DISTANCE_MAX:
            if min_range > DISTANCE_MIN:
                self.sound_period = M * min_range
                self.safety_factor = M_SAFETY * (min_range-DISTANCE_MIN)
            else:
                self.sound_period = MIN_PERIOD
                self.safety_factor = 0
        else:
            self.sound_period = MAX_PERIOD
            self.safety_factor = 1.0
        return

                    



        
if __name__ == '__main__':
    
    try:
        teleop_safety() 
    except:
        rospy.logwarn('Cannot run teleop_safety')