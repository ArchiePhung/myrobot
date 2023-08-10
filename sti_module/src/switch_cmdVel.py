#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Developer: Hoang van Quang
Company: STI Viet Nam
Date: 25/11/2020

"""
import roslib

import sys
import time
from decimal import *
import math
import rospy

from sti_msgs.msg import *
from geometry_msgs.msg import Twist
#--------------------------------------------------------------------------------- ROS
class switch_cmd():
	def __init__(self):
		rospy.init_node('switch_cmd', anonymous=False)
		self.rate = rospy.Rate(1)
		rospy.Subscriber("/cmd_vel_moveBase", Twist, self.cmdMove_callback)

		self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=100)	
		self.cmd_vel = Twist()	

	def cmdMove_callback(self, dat):
		self.cmd_vel = dat
		self.pub_cmdVel.publish(self.cmd_vel)
	
		self.rate.sleep()

	def run(self):
		while not rospy.is_shutdown():
			self.rate.sleep()

def main():
	class_1 = switch_cmd()
	class_1.run()

if __name__ == '__main__':
	main()	
