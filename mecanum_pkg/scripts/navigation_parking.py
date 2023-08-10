#!/usr/bin/env python
# Author : Quang 06/01/2023

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist ,Pose ,Point
from sti_msgs.msg import *
import math
import time
import threading
import signal
import os
from math import atan2, sin, cos, sqrt, degrees, radians

class navigation_parking():
	def __init__(self):
		rospy.init_node('navigation_parking', anonymous = False)
		self.rate = rospy.Rate(100)

		self.tf_listener = tf.TransformListener()
		self.tf_broadcaster = tf.TransformBroadcaster()

		self.sub_info_marker = rospy.Subscriber('/tag_detections_d435_parking', AprilTagDetectionArray, self.callback_GetMarkerOdom, queue_size = 1)
		# --
		self.tagTarget_ID = 3
		self.tagTarget_nameTag = ''
		self.tagTarget_pose = Pose()
		# -
		self.saveTime_receivedTag = time.time()
		self.timeOut_lostTag = 1.0 # - 1 s.
		# -
		self.idTag_findNow_list = []
		

	def callback_GetMarkerOdom(self, markers_odom_msg): # 5Hz
		sl_tag = len(markers_odom_msg.detections)
		if sl_tag == 0 : 
			self.idTag_findNow_list = []
		else:
			self.idTag_findNow_list = []
			for sl_tag in range(sl_tag):            
				# - Check id
				ID = markers_odom_msg.detections[sl_tag-1].id[0] 
				self.idTag_findNow_list.append(ID)
				print ("List ID Find: ", self.idTag_findNow_list)

				# - Get ID Tag Target.
				self.tagTarget_nameTag = "tag_" + str(int(self.tagTarget_ID))
				if ID == self.tagTarget_ID:
					print ("-------------------")
					self.tf_listener.waitForTransform(self.tagTarget_nameTag, "base_footprint", rospy.Time(), rospy.Duration(1)) 
					position, quaternion = self.tf_listener.lookupTransform(self.tagTarget_nameTag, "base_footprint", rospy.Time())
					quaternion = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
					self.tagTarget_pose.position.x = position[0]
					self.tagTarget_pose.position.y = position[1]
					self.tagTarget_pose.position.z = position[2]

					self.tagTarget_pose.orientation.x = quaternion[0]
					self.tagTarget_pose.orientation.y = quaternion[1]
					self.tagTarget_pose.orientation.z = quaternion[2]
					self.tagTarget_pose.orientation.w = quaternion[3]

					theta = tf.transformations.euler_from_quaternion(quaternion)[1]
					theta_0 = tf.transformations.euler_from_quaternion(quaternion)[0]
					theta_2 = tf.transformations.euler_from_quaternion(quaternion)[2]
					# --
					print (self.tagTarget_pose.position)
					print ("theta: ", degrees(theta) )
					print ("theta_0: ", degrees(theta_0) )
					print ("theta_2: ", degrees(theta_2) )
					# print (self.tagTarget_pose)

	def quaternion_to_euler(self, qua):
		quat = (qua.x, qua.y, qua.z, qua.w )
		a, b, euler = euler_from_quaternion(quat)
		return euler

	def euler_to_quaternion(self, euler):
		quat = Quaternion()
		odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat[0]
		quat.y = odom_quat[1]
		quat.z = odom_quat[2]
		quat.w = odom_quat[3]
		return quat
		
	def run(self):
		while not rospy.is_shutdown():
			self.rate.sleep()

def main():
	print('Program starting')
	program = navigation_parking()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()