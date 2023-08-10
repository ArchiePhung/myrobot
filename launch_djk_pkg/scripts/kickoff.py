#!/usr/bin/env python3
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
		rospy.init_node('kickoff', anonymous=False)
		self.rate = rospy.Rate(10)

		self.count_node = 0.
		self.notification = ''
		self.step = 0
		self.timeWait = 0.4 # s

		self.pub_stausLaunch = rospy.Publisher('status_launch', Status_launch, queue_size= 10)
		self.stausLaunch = Status_launch()

		# - 1 - module - firstWork.
		self.path_firstWork = rospy.get_param('path_firstWork', '')
		self.launch_firstWork = Launch(self.path_firstWork)
		rospy.Subscriber('/first_work/run', Int16, self.callBack_firstWork)
		self.is_firstWork = 0
		self.count_node += 1

		# - 2 - module - checkPort.
		self.path_checkPort = rospy.get_param('path_checkPort', '')
		self.launch_checkPort = Launch(self.path_checkPort)
		rospy.Subscriber('/status_port', Status_port, self.callBack_checkPort)
		self.is_checkPort = 0
		self.count_node += 1

		# - 3 - module - reconnectAll.
		self.path_reconnectAll = rospy.get_param('path_reconnectAll', '')
		self.launch_reconnectAll = Launch(self.path_reconnectAll)
		rospy.Subscriber('/status_reconnect', Status_reconnect, self.callBack_reconnectAll)
		self.is_reconnectAll = 0
		self.count_node += 1

		# - 4 - module - Board - CAN.
		self.path_board = rospy.get_param('path_board', '')
		self.launch_board = Launch(self.path_board)
		rospy.Subscriber('/CAN_received', CAN_received, self.callBack_board)
		self.is_board = 0
		self.count_node += 1

		# - 5 - module - Board - convert CAN.
		self.path_convertCAN = rospy.get_param('path_convertCAN', '')
		self.launch_convertCAN = Launch(self.path_convertCAN)
		self.count_node += 1

		# - 6 - module - Motor - CAN.
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

		# -- module - IMU. raw_imu_bno055
		self.path_imu = rospy.get_param('path_imu', '')
		self.launch_imu = Launch(self.path_imu)
		rospy.Subscriber('/imu_version1', Imu_version1, self.callBack_imu)
		self.is_imu = 0
		self.count_node += 1

		# -- module - tf.
		self.path_tf = rospy.get_param('path_tf', '')
		self.launch_tf = Launch(self.path_tf)
		self.is_tf = 1
		self.count_node += 1

		# -- module - camera.
		self.path_camera = rospy.get_param('path_camera', '')
		self.launch_camera = Launch(self.path_camera)
		rospy.Subscriber('/camera/color/image_raw', Image, self.callBack_camera)
		self.is_camera = 0
		self.count_node += 1

		# -- module - kinematic.
		self.path_kinematic = rospy.get_param('path_kinematic', '')
		self.launch_kinematic = Launch(self.path_kinematic)
		rospy.Subscriber('/mecanum_request', Mecanum_request, self.callBack_kinematic)
		self.is_kinematic = 0
		self.count_node += 1

		# -- module - lidar.
		self.path_lidar = rospy.get_param('path_lidar', '')
		self.launch_lidarFull = Launch(self.path_lidar)
		rospy.Subscriber('/scan', LaserScan, self.callBack_lidarFull)
		self.is_lidarFull = 0
		self.count_node += 1

		# -- module - imuFilter.
		self.path_imuFilter = rospy.get_param('path_imuFilter', '')
		self.launch_imuFilter = Launch(self.path_imuFilter)
		rospy.Subscriber('/imu_version1', Imu_version1, self.callBack_imuFilter)
		self.is_imuFilter = 0
		self.count_node += 1

		# -- module odomEncoder.
		self.path_odomEncoder = rospy.get_param('path_odomEncoder', '')
		self.launch_odomEncoder = Launch(self.path_odomEncoder)
		rospy.Subscriber('/raw_odom', Odometry, self.callBack_odomEncoder)
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

		# -- module - Setpose.
		self.path_setPose = rospy.get_param('path_setPose', '')
		self.launch_setPose = Launch(self.path_setPose)
		rospy.Subscriber('/setpose_status', Setpose_status, self.callBack_setPose)
		self.is_setPose = 0
		self.count_node += 1

		# -- module - Parking.
		self.path_parking = rospy.get_param('path_parking', '')
		self.launch_parking = Launch(self.path_parking)
		rospy.Subscriber('/parking_status', Parking_status, self.callBack_parking)
		self.is_parking = 0
		self.count_node += 1

		# -- atuo launch file detect
		self.path_apriltag = rospy.get_param('path_apriltag', '')
		self.launch_apriltag = Launch(self.path_apriltag)
		# rospy.Subscriber('/parking_status', Parking_status, self.callBack_apriltag)
		self.is_apriltag = 0
		self.count_node += 1

		# -- module - safetyZone.
		self.path_safetyZone = rospy.get_param('path_safetyZone', '')
		self.launch_safetyZone = Launch(self.path_safetyZone)
		rospy.Subscriber("/safety_zone", Zone_lidar_2head, self.callBack_safetyZone)
		self.is_safetyZone = 0
		self.count_node += 1

		# -- module - navigation.
		self.path_navigation = rospy.get_param('path_navigation', '')
		self.launch_navigation = Launch(self.path_navigation)
		rospy.Subscriber('/status_goal_control', Status_goal_control, self.callBack_navigation)
		self.is_navigation = 0
		self.count_node += 1

		# -- module - webconsole.
		# self.path_webconsole = rospy.get_param('path_webconsole', '')
		# self.launch_webconsole = Launch(self.path_webconsole)
		# self.is_webconsole = 0
		# self.count_node += 1

		# -- module - client.
		self.path_client = rospy.get_param('path_client', '')
		self.launch_client = Launch(self.path_client)
		rospy.Subscriber('/NN_infoRequest', NN_infoRequest, self.callBack_client)
		self.is_client = 0
		self.count_node += 1

		# -- module - control.
		self.path_control = rospy.get_param('path_control', '')
		self.launch_control = Launch(self.path_control)
		rospy.Subscriber('/NN_infoRespond', NN_infoRespond, self.callBack_control)
		self.is_control = 0
		self.count_node += 1

		# -- module - debug.
		self.path_debug = rospy.get_param('path_debug', '')
		self.launch_debug = Launch(self.path_debug)
		# rospy.Subscriber('/', , self.callBack_debug)
		self.is_debug = 0
		self.count_node += 1

		# -- module - tagSetpose.
		# self.path_tagSetpose = rospy.get_param('path_tagSetpose', '')
		# self.launch_tagSetpose = Launch(self.path_tagSetpose)
		# rospy.Subscriber('/robot_pose', Pose, self.callBack_tagSetpose)
		# self.is_tagSetpose = 0
		# self.count_node += 1

		# -- module - tagParking.
		# self.path_tagParking = rospy.get_param('path_tagParking', '')
		# self.launch_tagParking = Launch(self.path_tagParking)
		# rospy.Subscriber('/robot_pose', Pose, self.callBack_tagParking)
		# self.is_tagParking = 0
		# self.count_node += 1

		# # -- module - checkShelves.
		# self.path_checkShelves = rospy.get_param('path_checkShelves', '')
		# self.launch_checkShelves = Launch(self.path_checkShelves)
		# rospy.Subscriber('/check_shelves', Check_shelves, self.callBack_checkShelves)
		# self.is_checkShelves = 0
		# self.count_node += 1

		# -- module - zone3d.
		# self.path_zone3d = rospy.get_param('path_zone3d', '')
		# self.launch_zone3d = Launch(self.path_zone3d)
		# rospy.Subscriber('/safety_zone', Safety_zone, self.callBack_zone3d)
		# self.is_zone3d = 0
		# self.count_node += 1

	def callBack_firstWork(self, data):
		self.is_firstWork = 1

	def callBack_checkPort(self, data):
		self.is_checkPort = 1

	def callBack_reconnectAll(self, data):
		self.is_reconnectAll = 1
	# -- --
	def callBack_board(self, data):
		self.is_board = 1

	def callBack_motor(self, data):
		self.is_motor = 1

	def callBack_driverControl(self, data):
		self.is_driverControl = 1

	def callBack_imu(self, data):
		self.is_imu = 1

	def callBack_camera(self, data):
		self.is_camera = 1

	def callBack_lidarFull(self, data):
		self.is_lidarFull = 1
	# -- --
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

	def callBack_ekf(self, data):
		self.is_ekf = 1

	def callBack_mapServer(self, data):
		self.is_mapServer = 1

	def callBack_amcl(self, data):
		self.is_amcl = 1

	def callBack_posePublisher(self, data):
		self.is_posePublisher = 1

	def callBack_setPose(self, data):
		self.is_setPose = 1

	def callBack_parking(self, data):
		self.is_parking = 1
	# --
	def callBack_safetyZone(self, data):
		self.is_safetyZone = 1

	def callBack_navigation(self, data):
		self.is_navigation = 1

	def callBack_client(self, data):
		self.is_client = 1

	def callBack_control(self, data):
		self.is_control = 1

	def run(self):
		while not rospy.is_shutdown():
			# print "runing"
			# - 1 - firstWork
			if (self.step == 0):
				self.notification = 'launch_firstWork'
				self.launch_firstWork.start()
				if (self.is_firstWork == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# - 2 - CheckPort
			elif (self.step == 1):
				self.notification = 'launch_checkPort'
				self.launch_checkPort.start()
				if (self.is_checkPort == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# - 3 - Reconnect
			elif (self.step == 2):
				self.notification = 'launch_reconnectAll'
				# self.launch_reconnectAll.start()
				self.is_reconnectAll = 1
				if (self.is_reconnectAll == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# - 4 - Board
			elif (self.step == 5):
				self.notification = 'launch_RTC_BOARD'
				self.launch_board.start()
				if (self.is_board == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# - 5 - Board Convert CAN
			elif (self.step == 3):
				self.notification = 'launch_convertCAN'
				sts = self.launch_convertCAN.start_and_wait(2)
				if sts == 1:
					self.step += 1
					time.sleep(self.timeWait)

			# - 6 - Motor CAN
			elif (self.step == 4):
				self.notification = 'launch_motor'
				self.launch_motor.start()
				if (self.is_motor == 1):
					self.step += 1
					time.sleep(self.timeWait)
					
            # - 7 - Motor Control
			elif (self.step == 5):
				self.notification = 'launch_driverControl'
				self.launch_driverControl.start()
				if (self.is_driverControl == 1):
					self.step += 1
					time.sleep(self.timeWait)

            # - 8 - Kinematic
			elif (self.step == 6):
				self.notification = 'launch_kinematic'
				self.launch_kinematic.start()
				if (self.is_kinematic == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# - 9 - IMU
			elif (self.step == 6):
				self.notification = 'launch_imu'
				self.launch_imu.start()
				if (self.is_imu == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# - 10 - tf
			elif (self.step == 12):
				self.notification = 'launch_tf'
				sts = self.launch_tf.start_and_wait(4.)
				# if (self.is_tf == 1):
				if (sts == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- Camera
			elif (self.step == 13):
				self.notification = 'launch_camera'
				self.launch_camera.start()
				if (self.is_camera == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- Lidar
			elif (self.step == 14):
				self.notification = 'launch_lidarFull'
				self.launch_lidarFull.start()
				if (self.is_lidarFull == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- imuFilter
			elif (self.step == 16):
				self.notification = 'launch_imuFilter'
				self.launch_imuFilter.start()
				if (self.is_imuFilter == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- odomEncoder
			elif (self.step == 17):
				self.notification = 'launch_odomEncoder'
				self.launch_odomEncoder.start()
				if (self.is_odomEncoder == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- odomHector
			elif (self.step == 18):
				self.notification = 'launch_odomHector'
				self.launch_odomHector.start()
				if (self.is_odomHector == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- poseLidar
			elif (self.step == 19):
				self.notification = 'launch_poseLidar'
				self.launch_poseLidar.start()
				if (self.is_poseLidar == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- ekf
			elif (self.step == 20):
				self.notification = 'launch_ekf'
				self.launch_ekf.start()
				if (self.is_ekf == 1):
					self.step += 1
					# self.step = 19
					time.sleep(self.timeWait)

			# -- mapServer
			elif (self.step == 21):
				self.notification = 'launch_mapServer'
				self.launch_mapServer.start()
				if (self.is_mapServer == 1):
					self.step += 1
					# self.step = 19
					time.sleep(self.timeWait)

			# -- amcl
			elif (self.step == 22):
				self.notification = 'launch_amcl'
				self.launch_amcl.start()
				if (self.is_amcl == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- posePublisher
			elif (self.step == 23):
				self.notification = 'launch_posePublisher'
				self.launch_posePublisher.start()
				if (self.is_posePublisher == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- Setpose
			elif (self.step == 24):
				self.notification = 'launch_setPose'
				self.launch_setPose.start()
				if (self.is_setPose == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- Parking
			elif (self.step == 25):
				self.notification = 'launch_parking'
				self.launch_parking.start()
				if (self.is_parking == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- Auto launch detect
			elif (self.step == 26):
				self.notification = 'launch_apriltag'
				sts = self.launch_apriltag.start_and_wait(4.)
				if (self.is_apriltag == 1 or sts == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- safetyZone
			elif (self.step == 27):
				self.notification = 'launch_safetyZone'
				self.launch_safetyZone.start()
				if (self.is_safetyZone == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- navigation
			elif (self.step == 28):
				self.notification = 'launch_navigation'
				self.launch_navigation.start()
				if (self.is_navigation == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- webconsole
			# elif (self.step == 29):
			# 	self.notification = 'launch_webconsole'
			# 	sts = self.launch_webconsole.start_and_wait(3.)
			# 	if(sts == 1):
			# 		self.step += 1
			# 		time.sleep(self.timeWait)

			# -- client
			elif (self.step == 29):
				self.notification = 'launch_client'
				sts = self.launch_client.start_and_wait(3.)
				if (self.is_client == 1 or sts == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- control
			elif (self.step == 30):
				self.notification = 'launch_control'
				self.launch_control.start()
				if (self.is_control == 1):
					self.step += 1
					time.sleep(self.timeWait)

			# -- debug
			elif (self.step == 31):
				self.notification = 'launch_debug'
				sts = self.launch_debug.start_and_wait(4.)
				# if (self.is_debug == 1):
				if (sts == 1):
					self.step += 1
					time.sleep(self.timeWait)
		
			# -- Completed
			elif (self.step == 32):
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
