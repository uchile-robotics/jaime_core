#!/usr/bin/env python

import roslib
import rospy
from kobuki_msgs.msg import SensorState
from std_msgs.msg import UInt8

class kobuki_battery():
	kobuki_base_max_charge = 160

	def __init__(self):
		rospy.init_node('kobuki_battery')       
		rospy.Subscriber('/jaime/mobile_base/sensors/core', SensorState, self.BatteryCallback)

		rospy.spin()

	def BatteryCallback(self, data):
		battery_percent = round(float(data.battery) / float(self.kobuki_base_max_charge) * 100)
		pub = rospy.Publisher('/jaime/battery_percent', UInt8, queue_size = 10)
		pub.publish(battery_percent)
		rospy.loginfo(str(battery_percent))
		return

if __name__ == '__main__':
	try:
		kobuki_battery()
	except:
		rospy.loginfo('Cannot run battery info')

