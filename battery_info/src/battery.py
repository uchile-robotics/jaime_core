#!/usr/bin/env python

import roslib
import rospy
from kobuki_msgs.msg import SensorState
from std_msgs.msg import UInt8

class kobuki_battery():


	def __init__(self):
		rospy.init_node('kobuki_battery')
		self.Vmax    = rospy.get_param('~kobuki_base_max_charge', 165)
		self.Vmin    = rospy.get_param('~kobuki_base_min_charge', 130)   
		self.conv_factor = 100.0/(self.Vmax - self.Vmin)    
		rospy.Subscriber('/jaime/mobile_base/sensors/core', SensorState, self.BatteryCallback)
		self.pub = rospy.Publisher('/jaime/battery_percent', UInt8, queue_size = 10)

		rospy.spin()

	def BatteryCallback(self, data):
		battery_percent = round(float(data.battery-self.Vmin) * self.conv_factor )
		print(battery_percent)
		self.pub.publish(battery_percent)
		rospy.loginfo(str(battery_percent))
		return

if __name__ == '__main__':
	try:
		kobuki_battery()
	except:
		rospy.loginfo('Cannot run battery info')

