#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Authors : BEE
# DATE: 01/07/2021
# AUTHOR: HOANG VAN QUANG - BEE

import rospy
import sys
import time
import roslaunch
import os
import subprocess

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan , Image

from sti_msgs.msg import *
from message_pkg.msg import *
"""
yêu cầu thông tin để kết nối lại 1 node:
1, Topic để kiểm tra node đó có đang hoạt động không.
2, Tên node để shutdown node đó.
3, File launch khởi tạo node.
----------------
Nếu cổng vật lý vẫn còn.
"""

class Reconnect:
	def __init__(self, nameTopic_sub, time_checkLost, time_waitLaunch, file_launch):
		# -- parameter
		self.time_checkLost = time_checkLost
		self.time_waitLaunch = time_waitLaunch # wait after launch
		self.fileLaunch = file_launch
		self.nameTopic_sub = nameTopic_sub
		self.nameNode = ''
		self.is_nameReaded = 0
        # -- launch
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		# -- variable
		self.lastTime_waitConnect = time.time()
		self.time_readed = time.time()
		self.enable_check = 0
		self.process = 0
		self.numberReconnect = 0
		# -- 
		self.lastTime_waitShutdown = time.time()
		
	def read_nameNode(self, topic):
		try:
			output = subprocess.check_output("rostopic info {}".format(topic), shell= True)
			# print("out: ", output)

			pos1 = output.find('Publishers:') # tuyet doi ko sua linh tinh.
			pos2 = output.find(' (http://')   # tuyet doi ko sua linh tinh.
			# print("pos1: ", pos1)
			# print("pos2: ", pos2)
			if (pos1 >= 0):
				name = output[pos1 + 16 :pos2]
				p1 = name.find('Subscribers:')
				p2 = name.find('*')
				if (p1 == -1 and p2 == -1):
					return name
				else:
					print ("p1: ", p1)
					print ("p2: ", p2)
					print "Read name error: " + name + "|" + self.nameTopic_sub
					return ''
			else:
				# print ("Not!")
				return ''

		except Exception, e:
		    # print ("Error!")
			return ''

	def run_reconnect_vs2(self, enb_check, time_readed): # have kill node via name_topic pub.
		self.enable_check = enb_check
		self.time_readed = time_readed
		# -- read name node.
		if (enb_check == 1 and self.is_nameReaded == 0):
			self.nameNode = self.read_nameNode(self.nameTopic_sub)
			if (len(self.nameNode) != 0):
				self.is_nameReaded = 1
		# -- 	
		launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
		if (self.process == 0): # - Check lost.
			if (self.enable_check):
				t = time.time() - self.time_readed
				if (t >= self.time_checkLost):
					print ("Detected Lost: " + str(self.nameNode))
					self.process = 1

		elif (self.process == 1): # - shutdown node
			launch.shutdown()
			print ("shutdown node: ", str(self.nameNode))

			if (self.is_nameReaded):
				# self.nameNode = "/lineMagnetic"
				os.system("rosnode kill " + self.nameNode)
				print ("rosnode kill " + self.nameNode)

			self.lastTime_waitShutdown = time.time()
			print ("Wait after shutdown node: " + self.nameNode)
			self.process = 2

		elif (self.process == 2): # - wait after shutdown.
			t = time.time() - self.lastTime_waitShutdown
			if (t >= 3):
				self.process = 3

		elif (self.process == 3): # - Launch
			print ("Launch node: " + self.nameNode)
			launch.start()
			self.numberReconnect += 1
			self.lastTime_waitConnect = time.time()
			self.process = 4

		elif (self.process == 4): # - wait after launch
			t1 = time.time() - self.lastTime_waitConnect
			t2 = time.time() - self.time_readed # have topic pub
			if (t1 >= self.time_waitLaunch or t2 <= 1):
				# print ("t1: ", t1)
				# print ("t2: ", t2)
				print ("Launch node completed: " + self.nameNode)
				self.process = 0
				self.is_nameReaded = 0
				self.nameNode = ''

		return self.process, self.numberReconnect

	def run_reconnect_vs3(self, enb_check, time_readed, off_kill): # have kill node via name_topic pub. + Add funtion turn of rosnode kill
		self.enable_check = enb_check
		self.time_readed = time_readed
		# -- read name node.
		if (enb_check == 1 and self.is_nameReaded == 0):
			self.nameNode = self.read_nameNode(self.nameTopic_sub)
			if (len(self.nameNode) != 0):
				self.is_nameReaded = 1
		# -- 	
		launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
		if (self.process == 0): # - Check lost.
			if (self.enable_check):
				t = time.time() - self.time_readed
				if (t >= self.time_checkLost):
					print ("Detected Lost: " + str(self.nameNode))
					self.process = 1

		elif (self.process == 1): # - shutdown node
			launch.shutdown()
			print ("shutdown node: ", str(self.nameNode))

			if (off_kill == 0):
				if (self.is_nameReaded):
					# self.nameNode = "/lineMagnetic"
					os.system("rosnode kill " + self.nameNode)
					print ("rosnode kill " + self.nameNode)

			self.lastTime_waitShutdown = time.time()
			print ("Wait after shutdown node: " + self.nameNode)
			self.process = 2

		elif (self.process == 2): # - wait after shutdown.
			t = time.time() - self.lastTime_waitShutdown
			if (t >= 3):
				self.process = 3

		elif (self.process == 3): # - Launch
			print ("Launch node: " + self.nameNode)
			launch.start()
			self.numberReconnect += 1
			self.lastTime_waitConnect = time.time()
			self.process = 4

		elif (self.process == 4): # - wait after launch
			t1 = time.time() - self.lastTime_waitConnect
			t2 = time.time() - self.time_readed # have topic pub
			if (t1 >= self.time_waitLaunch or t2 <= 1):
				# print ("t1: ", t1)
				# print ("t2: ", t2)
				print ("Launch node completed: " + self.nameNode)
				self.process = 0
				self.is_nameReaded = 0
				self.nameNode = ''

		return self.process, self.numberReconnect

	def run_reconnect(self, enb_check, time_readed): 
		self.enable_check = enb_check
		self.time_readed = time_readed

		launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
		if (self.process == 0): # - Check lost.
			if (self.enable_check):
				t = time.time() - self.time_readed
				if (t >= self.time_checkLost):
					print ("Detected Lost: " + str(self.nameNode))
					self.process = 1

		elif (self.process == 1): # - shutdown node
			launch.shutdown()
			print ("shutdown node: ", str(self.nameNode))
			self.lastTime_waitShutdown = time.time()

			print ("Wait after shutdown node: " + self.nameNode)
			self.process = 2

		elif (self.process == 2): # - wait after shutdown.
			t = time.time() - self.lastTime_waitShutdown
			if (t >= 3):
				self.process = 3

		elif (self.process == 3): # - Launch
			print ("Launch node: " + self.nameNode)
			launch.start()
			self.numberReconnect += 1
			self.lastTime_waitConnect = time.time()
			self.process = 4

		elif (self.process == 4): # - wait after launch
			t1 = time.time() - self.lastTime_waitConnect
			t2 = time.time() - self.time_readed # have topic pub
			if (t1 >= self.time_waitLaunch or t2 <= 1):
				# print ("t1: ", t1)
				# print ("t2: ", t2)
				print ("Launch node completed: " + self.nameNode)
				self.process = 0
				
		return self.process, self.numberReconnect

class Reconnect_node():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('reconnect_try', anonymous=False)
		self.rate = rospy.Rate(100)
		# -- Port
		rospy.Subscriber("/status_port", Status_port, self.callback_port)
		self.port_status = Status_port()
		# -- PUB
		self.pub_statusReconnect = rospy.Publisher('/status_reconnect', Status_reconnect, queue_size= 10)
		self.statusReconnect = Status_reconnect()
		self.pre_timePub = time.time()
		self.cycle_timePub = 0.1 # s
		# ---------------------
		# -- -- Board 
		self.path_main = rospy.get_param("path_main", '')
		self.timeLost_main = rospy.get_param("timeLost_main", 3)
		self.timeWait_main = rospy.get_param("timeWait_main", 5)
		self.topicSub_main = "/POWER_info"
		self.isRuned_main = 0
		self.timeReaded_main = time.time()
		self.nameNode_main = ''
		rospy.Subscriber(self.topicSub_main, POWER_info, self.callback_main)
		# -- .
		self.reconnect_main = Reconnect(self.topicSub_main, self.timeLost_main, self.timeWait_main, self.path_main)
		# ---------------------
		# -- -- IMU
		self.path_sc = rospy.get_param("path_sc", '')
		self.timeLost_sc = rospy.get_param("timeLost_sc", 3)
		self.timeWait_sc = rospy.get_param("timeWait_sc", 5)
		self.topicSub_sc = "/imu_version1"
		self.isRuned_sc = 0
		self.timeReaded_sc = time.time()
		self.nameNode_sc = ''
		rospy.Subscriber(self.topicSub_sc, Imu_version1, self.callback_sc)
		# -- .
		self.reconnect_sc = Reconnect(self.topicSub_sc, self.timeLost_sc, self.timeWait_sc, self.path_sc)
		# ---------------------
		# -- -- OC 
		self.path_oc = rospy.get_param("path_oc", '')
		self.timeLost_oc = rospy.get_param("timeLost_oc", 3)
		self.timeWait_oc = rospy.get_param("timeWait_oc", 5)
		self.topicSub_oc = "/lift_status"		
		self.isRuned_oc = 0
		self.timeReaded_oc = time.time()
		self.nameNode_oc = ''
		rospy.Subscriber(self.topicSub_oc, Lift_status, self.callback_oc)
		# -- .
		self.reconnect_oc = Reconnect(self.topicSub_oc, self.timeLost_oc, self.timeWait_oc, self.path_oc)
		# ---------------------
		# -- -- Camera
		self.path_camera = rospy.get_param("path_camera", '')
		self.timeLost_camera = rospy.get_param("timeLost_camera", 3)
		self.timeWait_camera = rospy.get_param("timeWait_camera", 5)
		self.topicSub_camera = "/camera/color/image_raw"		
		self.isRuned_camera = 0
		self.timeReaded_camera = time.time()
		self.nameNode_camera = ''
		rospy.Subscriber(self.topicSub_camera, Image, self.callback_camera)
		# -- .
		self.reconnect_camera = Reconnect(self.topicSub_camera, self.timeLost_camera, self.timeWait_camera, self.path_camera)
		# ---------------------
		# -- -- Lidar 
		self.path_lidar = rospy.get_param("path_lidar", '')
		self.timeLost_lidar = rospy.get_param("timeLost_lidar", 3)
		self.timeWait_lidar = rospy.get_param("timeWait_lidar", 5)
		self.topicSub_lidar = "/scan"		
		self.isRuned_lidar = 0
		self.timeReaded_lidar = time.time()
		self.nameNode_lidar = ''
		rospy.Subscriber(self.topicSub_lidar, LaserScan, self.callback_lidar)
		# -- .
		self.reconnect_lidar = Reconnect(self.topicSub_lidar, self.timeLost_lidar, self.timeWait_lidar, self.path_lidar)
		# ---------------------
		# -- -- Motor 
		self.path_driver1 = rospy.get_param("path_driverLeft", '')
		self.timeLost_driver1 = rospy.get_param("timeLost_driver1", 3)
		self.timeWait_driver1 = rospy.get_param("timeWait_driver1", 5)
		self.topicSub_driver1 = "/driver1_respond"
		self.isRuned_driver1 = 0
		self.timeReaded_driver1 = time.time()
		self.nameNode_driver1 = ''
		rospy.Subscriber(self.topicSub_driver1, Driver_respond, self.callback_driver1)
		# -- .
		self.reconnect_driver1 = Reconnect(self.topicSub_driver1, self.timeLost_driver1, self.timeWait_driver1, self.path_driver1)
		# ---------------------
		# -- -- Parking
		self.path_parking = rospy.get_param("path_parking", '')
		self.timeLost_parking = rospy.get_param("timeLost_parking", 3)
		self.timeWait_parking = rospy.get_param("timeWait_parking", 5)
		self.topicSub_parking = "/parking_status"		
		self.isRuned_parking = 0
		self.timeReaded_parking = time.time()
		self.nameNode_parking = ''
		rospy.Subscriber(self.topicSub_parking, Parking_status, self.callback_parking)
		# -- 
		self.reconnect_parking = Reconnect(self.topicSub_parking, self.timeLost_parking, self.timeWait_parking, self.path_parking)

	def callback_port(self, data):
		self.port_status = data

	def callback_hmi(self, data):
		self.isRuned_hmi = 1
		self.timeReaded_hmi = time.time()

	def callback_main(self, data):
		self.isRuned_main = 1
		self.timeReaded_main = time.time()

	def callback_mc(self, data):
		self.isRuned_mc = 1
		self.timeReaded_mc = time.time()

	def callback_sc(self, data):
		self.isRuned_sc = 1
		self.timeReaded_sc = time.time()

	def callback_oc(self, data):
		self.isRuned_oc = 1
		self.timeReaded_oc = time.time()

	def callback_hc(self, data):
		self.isRuned_hc = 1
		self.timeReaded_hc = time.time()

	def callback_magLine(self, data):
		self.isRuned_magLine = 1
		self.timeReaded_magLine = time.time()

	def callback_camera(self, data):
		self.isRuned_camera = 1
		self.timeReaded_camera = time.time()

	def callback_lidar(self, data):
		self.isRuned_lidar = 1
		self.timeReaded_lidar = time.time()

	def callback_driver1(self, data):
		self.isRuned_driver1 = 1
		self.timeReaded_driver1 = time.time()

	def callback_driver2(self, data):
		self.isRuned_driver2 = 1
		self.timeReaded_driver2 = time.time()
		
	def callback_parking(self, data):
		self.isRuned_parking = 1
		self.timeReaded_parking = time.time()

	def reconnect_node(self):
		# -- hmi
		process, num = self.reconnect_hmi.run_reconnect_vs2(self.isRuned_hmi, self.timeReaded_hmi)
		self.statusReconnect.hmi.sts = process
		self.statusReconnect.hmi.times = num
		# -- main
		process, num = self.reconnect_main.run_reconnect_vs2(self.isRuned_main, self.timeReaded_main)
		self.statusReconnect.main.sts = process
		self.statusReconnect.main.times = num
		# -- mc
		# process, num = self.reconnect_mc.run_reconnect_vs2(self.isRuned_mc, self.timeReaded_mc)
		# self.statusReconnect.mc.sts = process
		# self.statusReconnect.mc.times = num
		# -- sc
		process, num = self.reconnect_sc.run_reconnect_vs2(self.isRuned_sc, self.timeReaded_sc)
		self.statusReconnect.sc.sts = process
		self.statusReconnect.sc.times = num
		# -- oc
		# process, num = self.reconnect_oc.run_reconnect_vs2(self.isRuned_oc, self.timeReaded_oc)
		# self.statusReconnect.oc.sts = process
		# self.statusReconnect.oc.times = num
		# -- hc
		# process, num = self.reconnect_hc.run_reconnect_vs2(self.isRuned_hc, self.timeReaded_hc)
		# self.statusReconnect.hc.sts = process
		# self.statusReconnect.hc.times = num
		# -- magLine
		# process, num = self.reconnect_magLine.run_reconnect_vs2(self.isRuned_magLine, self.timeReaded_magLine) # 
		process, num = self.reconnect_magLine.run_reconnect_vs3(self.isRuned_magLine, self.timeReaded_magLine, 1) # turn off 'rosnode kill '
		self.statusReconnect.magLine.sts = process
		self.statusReconnect.magLine.times = num
		# -- camera
		process, num = self.reconnect_camera.run_reconnect_vs2(self.isRuned_camera, self.timeReaded_camera)
		self.statusReconnect.camera.sts = process
		self.statusReconnect.camera.times = num
		# -- lidar
		process, num = self.reconnect_lidar.run_reconnect_vs2(self.isRuned_lidar, self.timeReaded_lidar)
		self.statusReconnect.lidar.sts = process
		self.statusReconnect.lidar.times = num
		# -- driver1
		process, num = self.reconnect_driver1.run_reconnect_vs2(self.isRuned_driver1, self.timeReaded_driver1)
		self.statusReconnect.driver1.sts = process
		self.statusReconnect.driver1.times = num
		# -- driver2
		process, num = self.reconnect_driver2.run_reconnect_vs2(self.isRuned_driver2, self.timeReaded_driver2)
		self.statusReconnect.driver2.sts = process
		self.statusReconnect.driver2.times = num
		# -- parking
		process, num = self.reconnect_parking.run_reconnect(self.isRuned_parking, self.timeReaded_parking)
		self.statusReconnect.parking.sts = process
		self.statusReconnect.parking.times = num

		# -- -- -- 
		tim = (time.time() - self.pre_timePub)%60
		if (tim > self.cycle_timePub):
			self.pre_timePub = time.time()
			self.pub_statusReconnect.publish(self.statusReconnect)

	def run(self):
		while not rospy.is_shutdown():
			self.reconnect_node()

			self.rate.sleep()

		print('Programer stopped')

def main():
	print('Starting main program')

	program = Reconnect_node()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()
