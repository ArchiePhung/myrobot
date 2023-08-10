#!/usr/bin/env python
# Author: HOANG VAN QUANG - BEE
# DATE: 22/06/2021

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

from std_msgs.msg import Int16
from sti_msgs.msg import *
from message_pkg.msg import *

import roslaunch
import rospy
import string
import time
import os

class Launch:
    def __init__(self, file_launch):
        # -- parameter
        self.fileLaunch = file_launch
        # -- launch
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        # -- variable
        self.process = 0
        self.time_pre = time.time()

    def start(self):
        if (self.process == 0): # - Launch
            # print ("Launch node!")
            launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
            launch.start()
            self.process = 1  

    def start_and_wait(self, timeWait): # second - Dung cho cac node ko pub.
        if (self.process == 0): # - Launch
            # print ("Launch node!")
            launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.fileLaunch])
            launch.start()
            self.process = 1
            self.time_pre = time.time()
            return 0

        elif (self.process == 1): # - Wait
            t = (time.time() - self.time_pre)%60
            if (t > timeWait):
                self.process = 2
            return 0

        elif (self.process == 2): # - Wait
            return 1

class scanMap():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('scanMap_launch', anonymous=False)
		self.rate = rospy.Rate(10)

		self.count_node = 0
		self.notification = ''
		self.step = 0
		self.timeWait = 0.4 # s

		self.pub_stausLaunch = rospy.Publisher('status_launch', Status_launch, queue_size= 10)
		self.stausLaunch = Status_launch()

		# -- module - firstWork.
		self.path_firstWork = rospy.get_param('path_firstWork', '')
		self.launch_firstWork = Launch(self.path_firstWork)
		rospy.Subscriber('/first_work/run', Int16, self.callBack_firstWork)
		self.is_firstWork = 0
		self.count_node += 1

		# -- module - checkPort.
		self.path_checkPort = rospy.get_param('path_checkPort', '')
		self.launch_checkPort = Launch(self.path_checkPort)
		rospy.Subscriber('/status_port', Status_port, self.callBack_checkPort)
		self.is_checkPort = 0
		self.count_node += 1

		# -- module - Board - CAN.
		self.path_board = rospy.get_param('path_board', '')
		self.launch_board = Launch(self.path_board)
		rospy.Subscriber('/CAN_received', CAN_received, self.callBack_board)
		self.is_board = 0
		self.count_node += 1

		# -- module - Board - convert CAN.
		self.path_convertCAN = rospy.get_param('path_convertCAN', '')
		self.launch_convertCAN = Launch(self.path_convertCAN)
		self.count_node += 1

		# -- module - Motor - CAN.
		self.path_motor = rospy.get_param('path_motor', '')
		self.launch_motor = Launch(self.path_motor)
		rospy.Subscriber('/CAN_MOTOR_received', CAN_received, self.callBack_motor)
		self.is_motor = 0
		self.count_node += 1

		# -- module - Motor - Control.
		self.path_driver = rospy.get_param('path_driver', '')
		self.launch_driverControl = Launch(self.path_driver)
		rospy.Subscriber('/mecanum_respond', Mecanum_respond, self.callBack_driverControl)
		self.is_driverControl = 0
		self.count_node += 1

		# -- module - kinematic.
		self.path_kinematic = rospy.get_param('path_kinematic', '')
		self.launch_kinematic = Launch(self.path_kinematic)
		rospy.Subscriber('/mecanum_request', Mecanum_request, self.callBack_kinematic)
		self.is_kinematic = 0
		self.count_node += 1

		# -- module - tf scan.
		self.path_tfScan = rospy.get_param('path_tfScan', '')
		self.launch_tfScan = Launch(self.path_tfScan)
		# rospy.Subscriber('/driver2_respond', Driver_respond, self.callBack_driverRight)
		self.is_tfScan = 1
		self.count_node += 1

		# -- module - reconnectAll.
		# self.path_reconnectAll = rospy.get_param('path_reconnectAll', '')
		# self.launch_reconnectAll = Launch(self.path_reconnectAll)
		# rospy.Subscriber('/reconnect_status', , self.callBack_reconnectAll)
		# self.is_reconnectAll = 0
		# self.count_node += 1

		# -- module - lidar.
		self.path_lidarFull = rospy.get_param('path_lidarFull', '')
		self.launch_lidarFull = Launch(self.path_lidarFull)
		rospy.Subscriber('/scan', LaserScan, self.callBack_lidarFull)
		self.is_lidarFull = 0
		self.count_node += 1

		# -- module - safetyZone.
		self.path_safetyZone = rospy.get_param('path_safetyZone', '')
		self.launch_safetyZone = Launch(self.path_safetyZone)
		rospy.Subscriber('/safety_zone', Zone_lidar_2head, self.callBack_safetyZone)
		self.is_safetyZone = 0
		self.count_node += 1

		# -- module - naviManual.
		self.path_naviManual = rospy.get_param('path_naviManual', '')
		self.launch_naviManual = Launch(self.path_naviManual)
		rospy.Subscriber('/cmd_vel', Twist, self.callBack_naviManual)
		self.is_naviManual = 0
		self.count_node += 1

		# -- module - cartographer.
		self.path_cartographer = rospy.get_param('path_cartographer', '')
		self.launch_cartographer = Launch(self.path_cartographer)
		# rospy.Subscriber('/', Int16, self.callBack_cartographer)
		self.is_cartographer = 1
		self.count_node += 1

	def callBack_firstWork(self, data):
		self.is_firstWork = 1

	def callBack_checkPort(self, data):
		self.is_checkPort = 1

	def callBack_board(self, data):
		self.is_board = 1

	def callBack_motor(self, data):
		self.is_motor = 1

	def callBack_driverControl(self, data):
		self.is_driverControl = 1

	def callBack_reconnectAll(self, data):
		self.is_reconnectAll = 1

	def callBack_kinematic(self, data):
		self.is_kinematic = 1

	def callBack_lidarFull(self, data):
		self.is_lidarFull = 1

	def callBack_safetyZone(self, data):
		self.is_safetyZone = 1

	def callBack_naviManual(self, data):
		self.is_naviManual = 1

	def callBack_cartographer(self, data):
		self.is_cartographer = 1

	def run(self):
		while not rospy.is_shutdown():
			# -- firstWork
			if (self.step == 0):
			    self.notification = 'launch_firstWork'
			    self.launch_firstWork.start()
			    if (self.is_firstWork == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- checkPort
			elif (self.step == 1):
				self.notification = 'launch_checkPort'
				self.launch_checkPort.start()
				if (self.is_checkPort == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
			# -- Board
			elif (self.step == 2):
				self.notification = 'launch_board'
				self.launch_board.start()
				if (self.is_board == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- Board Convert CAN
			elif (self.step == 3):
				self.notification = 'launch_convertCAN'
				sts = self.launch_convertCAN.start_and_wait(2)
				if sts == 1:
					self.step += 1
					time.sleep(self.timeWait)

			# -- Motor CAN
			elif (self.step == 4):
				self.notification = 'launch_motor'
				self.launch_motor.start()
				if (self.is_motor == 1):
					self.step += 1
					time.sleep(self.timeWait)
					
            # -- Control Motor
			elif (self.step == 5):
				self.notification = 'launch_driverControl'
				self.launch_driverControl.start()
				if (self.is_driverControl == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- kinematic
			elif (self.step == 6):
				self.notification = 'launch_kinematic'
				self.launch_kinematic.start()
				if (self.is_kinematic == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- tf scan
			elif (self.step == 7):
				self.notification = 'launch_tfScan'
				self.launch_tfScan.start()
				if (self.is_tfScan == 1):
					print ("77")
					time.sleep(2)
					self.step += 1
					time.sleep(self.timeWait)

            # -- lidar
			elif (self.step == 8):
				print ("88")
				self.notification = 'launch_lidarFull'
				self.launch_lidarFull.start()
				if (self.is_lidarFull == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
            # -- safetyZone
			elif (self.step == 9):
				self.notification = 'launch_safetyZone'
				self.launch_safetyZone.start()
				if (self.is_safetyZone == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
            # -- naviManual
			elif (self.step == 10):
				self.notification = 'launch_naviManual'
				self.launch_naviManual.start()
				if (self.is_naviManual == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
            # -- cartographer
			elif (self.step == 11):
				# self.step += 1
				self.notification = 'launch_cartographer'
				self.launch_cartographer.start()
				if (self.is_cartographer == 1):
					self.step += 1
					time.sleep(self.timeWait)
            		
            # -- Completed
			elif (self.step == 12):
				self.notification = 'Completed!'
            	
            # -- -- PUBLISH STATUS
			self.stausLaunch.persent = (self.step/self.count_node)*100.
			self.stausLaunch.position = self.step
			self.stausLaunch.notification = self.notification
			self.pub_stausLaunch.publish(self.stausLaunch)
			# time.sleep(0.1)
			self.rate.sleep()

def main():
	print('Program starting')
	try:
		program = scanMap()
		program.run()
	except rospy.ROSInterruptException:
		pass
	print('Programer stopped')

if __name__ == '__main__':
	main()