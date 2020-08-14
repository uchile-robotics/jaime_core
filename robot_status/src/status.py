#!/usr/bin/env python

import roslib
import rospy
from kobuki_msgs.msg import SensorState, Sound
from std_msgs.msg import UInt8,Bool
from std_srvs.srv import Trigger, TriggerResponse

class simple_lowpass_filter():
        def __init__(self, alpha):
        	self.alpha = alpha
        	self.y_1 = None
        	self._call = self.set_init

        def set_init(self, x):
        	self.y_1 = x
        	self._call = self.update
        	return x

        def update(self, x):
        	y = self.y_1 + self.alpha*(x - self.y_1)
        	self.y_1 = y
        	return y

        def __call__(self, x):
        	return self._call(x)
        

class kobuki_status():


	def __init__(self):
		rospy.init_node('kobuki_status')
		# Parameters
		self.Vmax    = rospy.get_param('~kobuki_base_max_charge', 165)
		self.Vmin    = rospy.get_param('~kobuki_base_min_charge', 130)
		# Battery   
		self.conv_factor = 100.0/(self.Vmax - self.Vmin)
		self.lBat = 100
		# voltage filter
		self.filter = simple_lowpass_filter(alpha=1./8)
		rospy.Subscriber('/jaime/mobile_base/sensors/core', SensorState, self.BatteryCallback)
		self.pub = rospy.Publisher('/jaime/battery_percent', UInt8, queue_size = 10)
		self.ledon_pub = rospy.Publisher('/jaime/arduino/led/on', Bool, queue_size = 10, latch=True)
		self.sound_pub = rospy.Publisher('/jaime/mobile_base/commands/sound', Sound, queue_size = 10)
		rospy.sleep(1)
		self.ledon_pub.publish(True)
		# shutdown service proxy
		rospy.wait_for_service('shutdown_service')
		self.shutdown = rospy.ServiceProxy('shutdown_service', Trigger)
		rospy.spin()

	def BatteryCallback(self, data):
		volt = self.filter(data.battery)
		battery_percent = max(round(float(volt-self.Vmin) * self.conv_factor ),0)
		self.pub.publish(battery_percent)
		# 30% Threshold
		if battery_percent<=30 and self.lBat>30:
			self.sound_pub.publish(4)
		# 10% Threshold
		elif battery_percent<=10 and self.lBat>10:
			self.shutdown()
		self.lBat = battery_percent
		return

if __name__ == '__main__':
	#try:
	kobuki_status()
	#except:
	#	rospy.logwarn('Cannot run status')

