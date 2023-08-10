#!/usr/bin/env python
# Author: HOANG VAN QUANG - BEE
# DATE: 22/06/2021

from sensor_msgs.msg import LaserScan , Image , PointCloud2
from sensor_msgs.msg import Imu as Imu_SS
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import  OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
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
        
	def start(self):
		if (self.process == 0): # - Launch
			# print ("Launch node!")
			launch = roslaunch.parent.ROSLaunchParent(self.uuid , [self.fileLaunch])
			launch.start()
			self.process = 1  

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

		# -- module - reconnectAll.
		self.path_reconnectAll = rospy.get_param('path_reconnectAll', '')
		self.launch_reconnectAll = Launch(self.path_reconnectAll)
		# rospy.Subscriber('/reconnect_status', '', self.callBack_reconnectAll)
		self.is_reconnectAll = 1
		self.count_node += 1

		# -- module - Main.
		self.path_main = rospy.get_param('path_main', '')
		self.launch_main = Launch(self.path_main)
		rospy.Subscriber('/POWER_info', POWER_info, self.callBack_main)
		self.is_main = 0
		self.count_node += 1

		# -- module - SC. raw_imu_bno055
		self.path_sc = rospy.get_param('path_sc', '')
		self.launch_sc = Launch(self.path_sc)
		rospy.Subscriber('/imu_version1', Imu_version1, self.callBack_sc)
		self.is_sc = 0
		self.count_node += 1

		# -- module - driverLeft.
		self.path_driverLeft = rospy.get_param('path_driverLeft', '')
		self.launch_driverLeft = Launch(self.path_driverLeft)
		rospy.Subscriber('/driver1_respond', Driver_respond, self.callBack_driverLeft)
		self.is_driverLeft = 0
		self.count_node += 1

		# -- module - driverRight.
		self.path_driverRight = rospy.get_param('path_driverRight', '')
		self.launch_driverRight = Launch(self.path_driverRight)
		rospy.Subscriber('/driver2_respond', Driver_respond, self.callBack_driverRight)
		self.is_driverRight = 0
		self.count_node += 1

		# -- module - tf.
		self.path_tf = rospy.get_param('path_tf', '')
		self.launch_tf = Launch(self.path_tf)
		# rospy.Subscriber('/driver2_respond', Driver_respond, self.callBack_driverRight)
		self.is_tf = 1
		self.count_node += 1

		# -- module - lidar.
		self.path_lidarFull = rospy.get_param('path_lidarFull', '')
		self.launch_lidarFull = Launch(self.path_lidarFull)
		rospy.Subscriber('/scan', LaserScan, self.callBack_lidarFull)
		self.is_lidarFull = 0
		self.count_node += 1

		# -- module - kinematic.
		self.path_kinematic = rospy.get_param('path_kinematic', '')
		self.launch_kinematic = Launch(self.path_kinematic)
		rospy.Subscriber('/driver1_query', Driver_query, self.callBack_kinematic)
		self.is_kinematic = 0
		self.count_node += 1

		# -- module - imuFilter.
		self.path_imuFilter = rospy.get_param('path_imuFilter', '')
		self.launch_imuFilter = Launch(self.path_imuFilter)
		rospy.Subscriber('/imu/data_bno055', Imu_SS, self.callBack_imuFilter)
		self.is_imuFilter = 0
		self.count_node += 1

		# -- module odomEncoder.
		self.path_odomEncoder = rospy.get_param('path_odomEncoder', '')
		self.launch_odomEncoder = Launch(self.path_odomEncoder)
		rospy.Subscriber('/raw_vel', Velocities, self.callBack_odomEncoder)
		self.is_odomEncoder = 0
		self.count_node += 1

		# -- module - path_odomHector.
		self.path_odomHector = rospy.get_param('path_odomHector', '')
		self.launch_odomHector = Launch(self.path_odomHector)
		rospy.Subscriber('/scanmatch_odom', Odometry, self.callBack_odomHector)
		self.is_odomHector = 0
		self.count_node += 1

		# -- module - poseLidar.
		self.path_poseLidar = rospy.get_param('path_poseLidar', '')
		self.launch_poseLidar = Launch(self.path_poseLidar)
		rospy.Subscriber('/pose_lidar', PoseWithCovarianceStamped, self.callBack_poseLidar)
		self.is_poseLidar = 0
		self.count_node += 1

		# -- module - safetyZone.
		self.path_safetyZone = rospy.get_param('path_safetyZone', '')
		self.launch_safetyZone = Launch(self.path_safetyZone)
		rospy.Subscriber('/safety_zone', Safety_zone, self.callBack_safetyZone)
		self.is_safetyZone = 0
		self.count_node += 1

		# -- module - ekf.
		self.path_ekf = rospy.get_param('path_ekf', '')
		self.launch_ekf = Launch(self.path_ekf)
		rospy.Subscriber('/odom', Odometry, self.callBack_ekf)
		self.is_ekf = 0
		self.count_node += 1

		# -- module - mapServer.
		self.path_mapServer = rospy.get_param('path_mapServer', '')
		self.launch_mapServer = Launch(self.path_mapServer)
		rospy.Subscriber('/map', OccupancyGrid, self.callBack_mapServer)
		self.is_mapServer = 0
		self.count_node += 1

		# -- module - amcl.
		self.path_amcl = rospy.get_param('path_amcl', '')
		self.launch_amcl = Launch(self.path_amcl)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callBack_amcl)
		self.is_amcl = 0
		self.count_node += 1

		# -- module - posePublisher.
		self.path_posePublisher = rospy.get_param('path_posePublisher', '')
		self.launch_posePublisher = Launch(self.path_posePublisher)
		rospy.Subscriber('/robot_pose', Pose, self.callBack_posePublisher)
		self.is_posePublisher = 0
		self.count_node += 1

	def callBack_firstWork(self, data):
		self.is_firstWork = 1

	def callBack_checkPort(self, data):
		self.is_checkPort = 1

	def callBack_reconnectAll(self, data):
		self.is_reconnectAll = 1

	def callBack_main(self, data):
		self.is_main = 1

	def callBack_sc(self, data):
		self.is_sc = 1

	def callBack_driverLeft(self, data):
		self.is_driverLeft = 1

	def callBack_driverRight(self, data):
		self.is_driverRight = 1

	def callBack_lidarFull(self, data):
		self.is_lidarFull = 1

	def callBack_kinematic(self, data):
		self.is_kinematic = 1

	def callBack_imuFilter(self, data):
		self.is_imuFilter = 1

	def callBack_odomEncoder(self, data):
		self.is_odomEncoder = 1

	def callBack_odomHector(self, data):
		self.is_odomHector = 1

	def callBack_poseLidar(self, data):
		self.is_poseLidar = 1

	def callBack_safetyZone(self, data):
		self.is_safetyZone = 1

	def callBack_ekf(self, data):
		self.is_ekf = 1

	def callBack_mapServer(self, data):
		self.is_mapServer = 1

	def callBack_amcl(self, data):
		self.is_amcl = 1

	def callBack_posePublisher(self, data):
		self.is_posePublisher = 1

	def run(self):
		while not rospy.is_shutdown():
			# print "runing"
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

            # -- reconnectAll
			elif (self.step == 2):
				self.notification = 'launch_reconnectAll'
				# self.launch_reconnectAll.start()
				if (self.is_reconnectAll == 1):
					self.step += 1
					# self.step = 4
					time.sleep(self.timeWait)

			# -- main
			elif (self.step == 3):
				self.notification = 'launch_main'
				self.launch_main.start()
				if (self.is_main == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- sc
			elif (self.step == 4):
				self.notification = 'launch_sc'
				self.launch_sc.start()
				if (self.is_sc == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- driverLeft
			elif (self.step == 5):
				self.notification = 'launch_driverLeft'
				self.launch_driverLeft.start()
				if (self.is_driverLeft == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- driverRight
			elif (self.step == 6):
				self.notification = 'launch_driverRight'
				self.launch_driverRight.start()
				if (self.is_driverRight == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- tf
			elif (self.step == 7):
				self.notification = 'launch_tf'
				self.launch_tf.start()
				if (self.is_tf == 1):
					time.sleep(2)
					self.step += 1
					time.sleep(self.timeWait)

            # -- lidar
			elif (self.step == 8):
				self.notification = 'launch_lidarFull'
				self.launch_lidarFull.start()
				if (self.is_lidarFull == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- kinematic
			elif (self.step == 9):
				self.notification = 'launch_kinematic'
				self.launch_kinematic.start()
				if (self.is_kinematic == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- imuFilter
			elif (self.step == 10):
				self.notification = 'launch_imuFilter'
				self.launch_imuFilter.start()
				if (self.is_imuFilter == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- odomEncoder
			elif (self.step == 11):
				self.notification = 'launch_odomEncoder'
				self.launch_odomEncoder.start()
				if (self.is_odomEncoder == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- odomHector
			elif (self.step == 12):
				self.notification = 'launch_odomHector'
				self.launch_odomHector.start()
				if (self.is_odomHector == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- poseLidar
			elif (self.step == 13):
				self.notification = 'launch_poseLidar'
				self.launch_poseLidar.start()
				if (self.is_poseLidar == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- safetyZone
			elif (self.step == 14):
				self.notification = 'launch_safetyZone'
				self.launch_safetyZone.start()
				if (self.is_safetyZone == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- ekf
			elif (self.step == 15):
				self.notification = 'launch_ekf'
				self.launch_ekf.start()
				if (self.is_ekf == 1):
					self.step += 1
					# self.step = 19
					time.sleep(self.timeWait)

            # -- mapServer
			elif (self.step == 16):
				self.notification = 'launch_mapServer'
				self.launch_mapServer.start()
				if (self.is_mapServer == 1):
					self.step += 1
					# self.step = 19
					time.sleep(self.timeWait)

            # -- amcl
			elif (self.step == 17):
				self.notification = 'launch_amcl'
				self.launch_amcl.start()
				if (self.is_amcl == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- posePublisher
			elif (self.step == 18):
				self.notification = 'launch_posePublisher'
				self.launch_posePublisher.start()
				if (self.is_posePublisher == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # -- Completed
			elif (self.step == 19):
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