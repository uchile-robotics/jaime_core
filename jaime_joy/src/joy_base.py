#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import time
import numpy as np
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from jaime_joy import xbox
from std_msgs.msg import Bool


class JoystickBase(object):

    def __init__(self):
        rospy.loginfo('Joystick base init ...')

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_priority = rospy.Publisher('base/master_cmd_vel', Twist, queue_size=1)
        self.cancel_goal_client = rospy.ServiceProxy('/jaime/nav/goal_server/cancel', Empty)
        self.pub_neck = rospy.Publisher('servo', Int16, queue_size=1)

        # control
        self.ltime = 0
        self.ledon_pub_safety = rospy.Publisher('/jaime/arduino/led/safety',Bool,queue_size=2,latch=True)        
        self.ledon_pub_teleop = rospy.Publisher('/jaime/arduino/led/teleop',Bool,queue_size=2,latch=True)
        

        # control
        self.is_paused = False
        self.safety_layer = True

        # load configuration
        self.b_pause    = rospy.get_param('~b_pause', 'START')
        self.b_cancel   = rospy.get_param('~b_cancel', 'B')
        self.b_priority = rospy.get_param('~b_priority', 'BACK')
        self.b_neck_up   = rospy.get_param('~b_neck_up', 'UP')
        self.b_neck_down = rospy.get_param('~b_neck_down', 'DOWN')
        self.b_neck_reset = rospy.get_param('~b_neck_reset', 'RIGHT')
        self.max_neck_vel = rospy.get_param('~max_neck_vel', 2)
        self.max_neck_ang = rospy.get_param('~max_neck_ang', 180.0)
        self.min_neck_ang = rospy.get_param('~min_neck_ang', 0.0)
        self.neck_center_ang = rospy.get_param('~neck_center_ang', 90.0)

        
        a_linear   = rospy.get_param('~a_linear', 'LS_VERT')
        a_angular  = rospy.get_param('~a_angular', 'LS_HORZ')
        self.max_linear_vel  = rospy.get_param('~max_linear_vel', 0.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.5)
        
        key_mapper = xbox.KeyMapper()
        self.b_idx_pause    = key_mapper.get_button_id(self.b_pause)
        self.b_idx_cancel   = key_mapper.get_button_id(self.b_cancel)
        self.b_idx_priority = key_mapper.get_button_id(self.b_priority)
        self.b_idx_neck_up    = key_mapper.get_button_id(self.b_neck_up)
        self.b_idx_neck_down  = key_mapper.get_button_id(self.b_neck_down)
        self.b_idx_neck_reset = key_mapper.get_button_id(self.b_neck_reset)
        self.a_idx_linear   = key_mapper.get_axis_id(a_linear)
        self.a_idx_angular  = key_mapper.get_axis_id(a_angular)

        # Set neck pos
        self.neck_pos  = self.neck_center_ang

        # check
        self.assert_params()

        # ready to work
        rospy.Subscriber('joy', Joy, self.callback, queue_size=1)
        rospy.sleep(0.1)
        self.ledon_pub_safety.publish(self.safety_layer)
        self.ledon_pub_teleop.publish(not self.is_paused)
        rospy.loginfo('Joystick for base is ready')
        
    def assert_params(self):
        """
        checks whether the user parameters are valid or no.
        """
        assert isinstance(self.b_idx_pause, int)
        assert isinstance(self.a_idx_angular, int)
        assert isinstance(self.a_idx_linear, int)

    # this method breaks the decoupling between base and soft ws!!!
    def cancel_goal(self):
        try:
            self.cancel_goal_client.wait_for_service(0.5)
            self.cancel_goal_client()
            rospy.loginfo("Goal cancelled")
        except rospy.ServiceException:
            rospy.loginfo("There is no goal to cancel")
        except Exception:
            pass

    def move_neck(self):
        self.neck_pos = np.clip(self.neck_pos, self.min_neck_ang, self.max_neck_ang)
        cmd = Int16()
        cmd.data = int(self.neck_pos)
        self.pub_neck.publish(cmd)

    def callback(self, msg):

        # Safety Layer
        if msg.buttons[self.b_idx_priority]:
            self.safety_layer = not self.safety_layer
            self.ledon_pub_safety.publish(self.safety_layer)
            if self.safety_layer:
                rospy.logwarn("\n Safety layer activated!, press the " + self.b_priority + " button to resume it\n")
            else:
                rospy.logwarn("\n Safety layer paussed, press the " + self.b_priority + " button to resume it\n")
            rospy.sleep(1)
            return

        # pause
        if msg.buttons[self.b_idx_pause]:
            self.ledon_pub_teleop.publish(self.is_paused)
            self.is_paused = not self.is_paused
            if self.is_paused:

                # stop signal
                cmd = Twist()
                self.pub.publish(cmd)

                rospy.logwarn("\nControlling PAUSED!, press the " + self.b_pause + " button to resume it\n")
            else:
                rospy.logwarn("Controlling RESUMED, press " + self.b_pause + " button to pause it")

            # very important sleep!
            # prevents multiple triggersfor the same button
            rospy.sleep(1)  # it should be >= 1;
            return

        elif msg.buttons[self.b_idx_cancel]:
            self.cancel_goal()
            return

        # work
        if not self.is_paused:
            cmd = Twist()
            cmd.angular.z = self.max_angular_vel * msg.axes[self.a_idx_angular]
            cmd.linear.x = self.max_linear_vel * msg.axes[self.a_idx_linear]

            if self.safety_layer:
                self.pub.publish(cmd)
                
            else:
                self.pub_priority.publish(cmd)

            dt = time.time() - self.ltime
            if msg.buttons[self.b_idx_neck_up]:
                self.neck_pos += min(self.max_neck_vel * dt, self.max_neck_vel)
                self.move_neck()
            elif msg.buttons[self.b_idx_neck_down]:
                self.neck_pos -= min(self.max_neck_vel * dt, self.max_neck_vel)
                self.move_neck()
            elif msg.buttons[self.b_idx_neck_reset]:
                self.neck_pos = self.neck_center_ang
                self.move_neck()
            self.ltime = time.time()

if __name__ == '__main__':
    rospy.init_node('joy_base')
    JoystickBase()
    rospy.spin()
