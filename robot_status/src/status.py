#!/usr/bin/env python

import roslib
import rospy
from kobuki_msgs.msg import SensorState, Sound
from std_msgs.msg import UInt8,Bool

class kobuki_status():


	def __init__(self):
		rospy.init_node('kobuki_status')
		# Parameters
		self.Vmax    = rospy.get_param('~kobuki_base_max_charge', 165)
		self.Vmin    = rospy.get_param('~kobuki_base_min_charge', 130)
		# Battery   
		self.conv_factor = 100.0/(self.Vmax - self.Vmin)
		self.lBat = 100    
		rospy.Subscriber('/jaime/mobile_base/sensors/core', SensorState, self.BatteryCallback)
		self.pub = rospy.Publisher('/jaime/battery_percent', UInt8, queue_size = 10)
		self.ledon_pub = rospy.Publisher('/jaime/arduino/led/on', Bool, queue_size = 10, latch=True)
		self.sound_pub = rospy.Publisher('/jaime/mobile_base/commands/sound', Sound, queue_size = 10)
		rospy.sleep(1)
		self.ledon_pub.publish(True)
		rospy.spin()

	def BatteryCallback(self, data):
		battery_percent = round(float(data.battery-self.Vmin) * self.conv_factor )
		self.pub.publish(battery_percent)
		# 30% Threshold
		if battery_percent<=30 and self.lBat>30:
			self.sound_pub.publish(4)
		self.lBat = battery_percent
		return

if __name__ == '__main__':
	try:
		kobuki_status()
	except:
		rospy.logwarn('Cannot run status')

