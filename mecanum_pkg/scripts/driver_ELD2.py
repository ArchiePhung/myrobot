#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: HOANG VAN QUANG - BEE
# DATE: 03/08/2022

# from message_pkg.msg import *
from message_pkg.msg import CAN_send, CAN_status, CAN_received, Mecanum_respond, Mecanum_request
from sti_msgs.msg import *
from geometry_msgs.msg import Twist
import time
import rospy

class CAN_ROS():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('CAN_ROS', anonymous=False)
		self.rate = rospy.Rate(100)

		# ------------- CAN ------------- #
		# rospy.Subscriber("/CAN_status", CAN_status, self.statusCAN_callback)
		# self.CAN_status = CAN_status()

		rospy.Subscriber("/CAN_MOTOR_received", CAN_received, self.CAN_callback)
		self.data_receivedCAN = CAN_received()

		self.pub_sendCAN = rospy.Publisher("/CAN_MOTOR_send", CAN_send, queue_size= 10)
		self.data_sendCan = CAN_send()
		self.frequence_sendCAN = 24. # - Hz
		self.saveTime_sendCAN = time.time()
		self.sort_send = 1
		self.switch_id = 0

		# -------------  ------------- #
		rospy.Subscriber("/mecanum_request", Mecanum_request, self.mecanumRequest_callback)
		self.driver_control = Mecanum_request()

		self.pub_driverRespond = rospy.Publisher("/mecanum_respond", Mecanum_respond, queue_size = 20)
		self.driver_respond = Mecanum_respond()

		# ------------- VAR ------------- #
		self.revert = [0, 1, 0, 1]
		self.valueDirection = [0, 100, 0, 100]
		self.valueSpeed = [0, 0, 0, 0]
		# -
		self.valueDirection_set = [0, 100, 0, 100]
		self.valueSpeed_set = [0, 0, 0, 0]

		self.valueDirection_check = [0, 0, 0, 0]

		print ("--d---")
	def mecanumRequest_callback(self, data):
		self.driver_control = data
		self.controlDriection()

	def CAN_callback(self, dat):
		self.data_receivedCAN = dat
		# -- RECEIVED CAN
		self.receivedCAN_run()

	def statusCAN_callback(self, dat):
		self.CAN_status = dat

	def convert_4byte_int(self, byte0, byte1, byte2, byte3):
		int_out = 0
		int_out = byte0 + byte1*256 + byte2*256*256 + byte3*256*256*256
		if int_out > pow(2, 32)/2.:
			int_out = int_out - pow(2, 32)
		return int_out

	def getBit_fromInt8(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(8):
			bit_out = value_now%2
			value_now = value_now/2
			if (i == pos):
				return bit_out

			if (value_now < 1):
				return 0		
		return 0

	def int_to_4bytes(self, value): # => STT: [byte No.0, byte No.1, byte No.2, byte No.3]
		value_now = 0
		if (value >= 0):
			value_now = value
		else:
			value_now = (256**4) + value

		byteArray = [0, 0, 0, 0]
		byteArray[3] = int( value_now/(256**3) )
		byteArray[2] = int( (value_now - byteArray[3]*(256**3))/256**2 )
		byteArray[1] = int( (value_now - byteArray[3]*(256**3) - byteArray[2]*(256**2) )/256 )
		byteArray[0] = int( value_now - byteArray[3]*(256**3) - byteArray[2]*(256**2) - byteArray[1]*256 )
		return byteArray

	def controlDriection(self):
		self.valueSpeed[0] = abs(self.driver_control.speed1)
		self.valueSpeed[1] = abs(self.driver_control.speed2)
		self.valueSpeed[2] = abs(self.driver_control.speed3)
		self.valueSpeed[3] = abs(self.driver_control.speed4)
		# -- Driver 1
		if self.driver_control.speed1 > 0:
			self.valueDirection_check[0] = 1
			if self.revert[0] == 0:
				self.valueDirection[0] = 0
				
			else:
				self.valueDirection[0] = 100
		else:
			self.valueDirection_check[0] = 0
			if self.revert[0] == 0:
				self.valueDirection[0] = 100
			else:
				self.valueDirection[0] = 0

		# -- Driver 2
		if self.driver_control.speed2 > 0:
			self.valueDirection_check[1] = 1
			if self.revert[1] == 0:
				self.valueDirection[1] = 0
			else:
				self.valueDirection[1] = 100
		else:
			self.valueDirection_check[1] = 0
			if self.revert[1] == 0:
				self.valueDirection[1] = 100
			else:
				self.valueDirection[1] = 0

		# -- Driver 3
		if self.driver_control.speed3 > 0:
			self.valueDirection_check[2] = 1
			if self.revert[2] == 0:
				self.valueDirection[2] = 0
			else:
				self.valueDirection[2] = 100
		else:
			self.valueDirection_check[2] = 0
			if self.revert[2] == 0:
				self.valueDirection[2] = 100
			else:
				self.valueDirection[2] = 0

		# -- Driver 4
		if self.driver_control.speed4 > 0:
			self.valueDirection_check[3] = 1
			if self.revert[3] == 0:
				self.valueDirection[3] = 0
			else:
				self.valueDirection[3] = 100
		else:
			self.valueDirection_check[3] = 0
			if self.revert[3] == 0:
				self.valueDirection[3] = 100
			else:
				self.valueDirection[3] = 0
		
		# print ("Dir: ", self.valueDirection_check)

	def old_controlDriection(self, speed1, speed2, speed3, speed4):
		if (self.cmdVel.linear.x > 0 and self.cmdVel.linear.y == 0 and self.cmdVel.angular.z == 0):
			self.valueSpeed = [speed1, speed2, speed3, speed4]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 0
			else:
				self.valueDirection[0] = 100

			if (self.revert[1] == 0):
				self.valueDirection[1] = 0
			else:
				self.valueDirection[1] = 100

			if (self.revert[2] == 0):
				self.valueDirection[2] = 0
			else:
				self.valueDirection[2] = 100

			if (self.revert[3] == 0):
				self.valueDirection[3] = 0
			else:
				self.valueDirection[3] = 100
			# print ("-- Tien -- ")

		elif (self.cmdVel.linear.x < 0 and self.cmdVel.linear.y == 0 and self.cmdVel.angular.z == 0):
			self.valueSpeed = [speed1, speed2, speed3, speed4]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 100
			else:
				self.valueDirection[0] = 0

			if (self.revert[1] == 0):
				self.valueDirection[1] = 100
			else:
				self.valueDirection[1] = 0

			if (self.revert[2] == 0):
				self.valueDirection[2] = 100
			else:
				self.valueDirection[2] = 0

			if (self.revert[3] == 0):
				self.valueDirection[3] = 100
			else:
				self.valueDirection[3] = 0
			# print ("-- Lui -- ")

		elif (self.cmdVel.linear.x == 0 and self.cmdVel.linear.y > 0 and self.cmdVel.angular.z == 0):
			self.valueSpeed = [speed1, speed2, speed3, speed4]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 100
			else:
				self.valueDirection[0] = 0

			if (self.revert[1] == 0):
				self.valueDirection[1] = 0
			else:
				self.valueDirection[1] = 100

			if (self.revert[2] == 0):
				self.valueDirection[2] = 0
			else:
				self.valueDirection[2] = 100

			if (self.revert[3] == 0):
				self.valueDirection[3] = 100
			else:
				self.valueDirection[3] = 0
			# print ("-- Trai -- ")

		elif (self.cmdVel.linear.x == 0 and self.cmdVel.linear.y < 0 and self.cmdVel.angular.z == 0):
			self.valueSpeed = [speed1, speed2, speed3, speed4]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 0
			else:
				self.valueDirection[0] = 100

			if (self.revert[1] == 0):
				self.valueDirection[1] = 100
			else:
				self.valueDirection[1] = 0

			if (self.revert[2] == 0):
				self.valueDirection[2] = 100
			else:
				self.valueDirection[2] = 0

			if (self.revert[3] == 0):
				self.valueDirection[3] = 0
			else:
				self.valueDirection[3] = 100
			# print ("-- Phai -- ")

		# Cheo Trai - Tien
		elif (self.cmdVel.linear.x > 0 and self.cmdVel.linear.y > 0 and self.cmdVel.angular.z == 0):
			self.valueSpeed = [0, speed2, speed3, 0]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 0
			else:
				self.valueDirection[0] = 100

			if (self.revert[1] == 0):
				self.valueDirection[1] = 0
			else:
				self.valueDirection[1] = 100

			if (self.revert[2] == 0):
				self.valueDirection[2] = 0
			else:
				self.valueDirection[2] = 100

			if (self.revert[3] == 0):
				self.valueDirection[3] = 0
			else:
				self.valueDirection[3] = 100
			# print ("-- Trai - Tien -- ")

		# - Cheo Phai - Tien
		elif (self.cmdVel.linear.x > 0 and self.cmdVel.linear.y < 0 and self.cmdVel.angular.z == 0):
			self.valueSpeed = [speed, 0, 0, speed]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 0
			else:
				self.valueDirection[0] = 100

			if (self.revert[1] == 0):
				self.valueDirection[1] = 0
			else:
				self.valueDirection[1] = 100

			if (self.revert[2] == 0):
				self.valueDirection[2] = 0
			else:
				self.valueDirection[2] = 100

			if (self.revert[3] == 0):
				self.valueDirection[3] = 0
			else:
				self.valueDirection[3] = 100
			# print ("-- Phai - Tien -- ")

		# Cheo Trai - Lui
		elif (self.cmdVel.linear.x < 0 and self.cmdVel.linear.y > 0 and self.cmdVel.angular.z == 0):
			self.valueSpeed = [speed, 0, 0, speed]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 100
			else:
				self.valueDirection[0] = 0

			if (self.revert[1] == 0):
				self.valueDirection[1] = 0
			else:
				self.valueDirection[1] = 100

			if (self.revert[2] == 0):
				self.valueDirection[2] = 0
			else:
				self.valueDirection[2] = 100

			if (self.revert[3] == 0):
				self.valueDirection[3] = 100
			else:
				self.valueDirection[3] = 0
			# print ("-- Trai - Lui -- ")

		# Cheo Phai - Lui
		elif (self.cmdVel.linear.x < 0 and self.cmdVel.linear.y < 0 and self.cmdVel.angular.z == 0):
			self.valueSpeed = [0, speed, speed, 0]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 0
			else:
				self.valueDirection[0] = 100

			if (self.revert[1] == 0):
				self.valueDirection[1] = 100
			else:
				self.valueDirection[1] = 0

			if (self.revert[2] == 0):
				self.valueDirection[2] = 100
			else:
				self.valueDirection[2] = 0

			if (self.revert[3] == 0):
				self.valueDirection[3] = 0
			else:
				self.valueDirection[3] = 100
			# print ("-- Phai - Lui -- ")

		# Xoay Trai
		elif (self.cmdVel.linear.x == 0 and self.cmdVel.linear.y == 0 and self.cmdVel.angular.z > 0):
			self.valueSpeed = [10000, 10000, 10000, 10000]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 100
			else:
				self.valueDirection[0] = 0

			if (self.revert[1] == 0):
				self.valueDirection[1] = 0
			else:
				self.valueDirection[1] = 100

			if (self.revert[2] == 0):
				self.valueDirection[2] = 100
			else:
				self.valueDirection[2] = 0

			if (self.revert[3] == 0):
				self.valueDirection[3] = 0
			else:
				self.valueDirection[3] = 100
			# print ("-- Xoay trai -- ")

		elif (self.cmdVel.linear.x == 0 and self.cmdVel.linear.y == 0 and self.cmdVel.angular.z < 0):
			self.valueSpeed = [10000, 10000, 10000, 10000]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 0
			else:
				self.valueDirection[0] = 100

			if (self.revert[1] == 0):
				self.valueDirection[1] = 100
			else:
				self.valueDirection[1] = 0

			if (self.revert[2] == 0):
				self.valueDirection[2] = 0
			else:
				self.valueDirection[2] = 100

			if (self.revert[3] == 0):
				self.valueDirection[3] = 100
			else:
				self.valueDirection[3] = 0
			# print ("-- Tien -- ")

		else:
			self.valueSpeed = [0, 0, 0, 0]
			if (self.revert[0] == 0):
				self.valueDirection[0] = 100
			else:
				self.valueDirection[0] = 0

			if (self.revert[1] == 0):
				self.valueDirection[1] = 100
			else:
				self.valueDirection[1] = 0

			if (self.revert[2] == 0):
				self.valueDirection[2] = 0
			else:
				self.valueDirection[2] = 100

			if (self.revert[3] == 0):
				self.valueDirection[3] = 0
			else:
				self.valueDirection[3] = 100

			print ("-- Dung -- ")

	def sendCAN_All(self):
	 # ------------------ Start ------------------ #
		if (self.sort_send == 1): # -- Reset all nodes
			print ("-- 1 --")
			self.data_sendCan.id = 0
			self.data_sendCan.byte0 = 129
			self.data_sendCan.byte1 = 0
			self.data_sendCan.byte2 = 0
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 0
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 2

		elif (self.sort_send == 2): # -- Start remote control
			print ("-- 2 --")
			self.data_sendCan.id = 0
			self.data_sendCan.byte0 = 1
			self.data_sendCan.byte1 = 0
			self.data_sendCan.byte2 = 0
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 0
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 31

	 # ------------------ Direction ------------------ #
		elif (self.sort_send == 31): # -- Dir 1
			self.data_sendCan.id = 1537
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection[0] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 32

		elif (self.sort_send == 32): # -- Dir 2
			self.data_sendCan.id = 1538
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection[1] # 64h - 100
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 33

		elif (self.sort_send == 33): # -- Dir 3
			self.data_sendCan.id = 1539
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection[2] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 34

		elif (self.sort_send == 34): # -- Dir 4
			self.data_sendCan.id = 1540
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection[3] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 41
		
	 # ------------------ Switch On 6h ------------------ #
		elif (self.sort_send == 41): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 6 # 6
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 42

		elif (self.sort_send == 42): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 6 # 6
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 43

		elif (self.sort_send == 43): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 6 # 6
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 44

		elif (self.sort_send == 44): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 6 # 6
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 51

	 # ------------------ Switch On 7h ------------------ #
		elif (self.sort_send == 51): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 7 # 7
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 52

		elif (self.sort_send == 52): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 7 # 7
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 53

		elif (self.sort_send == 53): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 7 # 7
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 54

		elif (self.sort_send == 54): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 7 # 7
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 201

	 # ------------------ Write Reset Driver + On ------------------ #
		elif (self.sort_send == 201): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 128 # 
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 202

		elif (self.sort_send == 202): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 128 # 
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 203

		elif (self.sort_send == 203): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 128 # 
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 204

		elif (self.sort_send == 204): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 128 # 
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 61

	 # ------------------ Write Reset Driver + Off ------------------ #

	 # ------------------ Enable Driver Fh ------------------ #
		elif (self.sort_send == 61): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 15 # Fh
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 62

		elif (self.sort_send == 62): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 15 # Fh
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 63

		elif (self.sort_send == 63): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 15 # Fh
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 64

		elif (self.sort_send == 64): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 15 # Fh
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 71

	 # ------------------ Write operation mode ------------------ #
		elif (self.sort_send == 71): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 47
			self.data_sendCan.byte1 = 96
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 3
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 72

		elif (self.sort_send == 72): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 47
			self.data_sendCan.byte1 = 96
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 3
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 73

		elif (self.sort_send == 73): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 47
			self.data_sendCan.byte1 = 96
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 3
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 74

		elif (self.sort_send == 74): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 47
			self.data_sendCan.byte1 = 96
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 3
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 81

	 # ------------------ Protocol Acceleration ------------------ #
		elif (self.sort_send == 81): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 131
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(50000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 82

		elif (self.sort_send == 82): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 131
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(50000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 83

		elif (self.sort_send == 83): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 131
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(50000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 84

		elif (self.sort_send == 84): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 131
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(50000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 91

	 # ------------------ Protocol Deceleration ------------------ #
		elif (self.sort_send == 91): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 132
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(500000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 92

		elif (self.sort_send == 92): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 132
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(500000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 93

		elif (self.sort_send == 93): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 132
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(500000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 94

		elif (self.sort_send == 94): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 132
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(500000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 101

	 # ------------------ Protocol Write Speed ------------------ #
		elif (self.sort_send == 101): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0

			byte_arr = self.int_to_4bytes(self.valueSpeed[0])
			print ("valueSpeed: ", self.valueSpeed)
			# print ("byte_arr: ", byte_arr)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 102

		elif (self.sort_send == 102): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes( self.valueSpeed[1])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 103

		elif (self.sort_send == 103): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(self.valueSpeed[2])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 104

		elif (self.sort_send == 104): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(self.valueSpeed[3])
			# print ("byte_arr: ", byte_arr)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 111

	 # ------------------ Protocol Read Speed ------------------ #
		elif (self.sort_send == 111): # -- P28
			self.data_sendCan.id = 1537
			self.data_sendCan.byte0 = 64 	# 40h - 64
			self.data_sendCan.byte1 = 108 	# 6Ch
			self.data_sendCan.byte2 = 96  	# 60h
			self.data_sendCan.byte3 = 0   	# 
			self.data_sendCan.byte4 = 0 	# 
			self.data_sendCan.byte5 = 0 	# 
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 112

		elif (self.sort_send == 112): # -- 
			self.data_sendCan.id = 1538
			self.data_sendCan.byte0 = 64 	# 40
			self.data_sendCan.byte1 = 108 	# 6Ch
			self.data_sendCan.byte2 = 96  	# 60h
			self.data_sendCan.byte3 = 0   	# 
			self.data_sendCan.byte4 = 0 	# 
			self.data_sendCan.byte5 = 0 	# 
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 113

		elif (self.sort_send == 113): # -- 
			self.data_sendCan.id = 1539
			self.data_sendCan.byte0 = 64 	# 40
			self.data_sendCan.byte1 = 108 	# 6Ch
			self.data_sendCan.byte2 = 96  	# 60h
			self.data_sendCan.byte3 = 0   	# 
			self.data_sendCan.byte4 = 0 	# 
			self.data_sendCan.byte5 = 0 	# 
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 114

		elif (self.sort_send == 114): # -- 
			self.data_sendCan.id = 1540
			self.data_sendCan.byte0 = 64 	# 40
			self.data_sendCan.byte1 = 108 	# 6Ch
			self.data_sendCan.byte2 = 96  	# 60h
			self.data_sendCan.byte3 = 0   	# 
			self.data_sendCan.byte4 = 0 	# 
			self.data_sendCan.byte5 = 0 	# 
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 121

	 # ------------------ Protocol Dir Loop ------------------ #
		elif (self.sort_send == 121): # -- Dir 1
			for i in range(4):
				self.valueDirection_set[i] = self.valueDirection[i]
				self.valueSpeed_set[i] = self.valueSpeed[i]

			self.data_sendCan.id = 1537
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection_set[0] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 122

		elif (self.sort_send == 122): # -- Dir 3
			self.data_sendCan.id = 1539
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection_set[2] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 123

		elif (self.sort_send == 123): # -- Dir 4
			self.data_sendCan.id = 1540
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection_set[3] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 124

		elif (self.sort_send == 124): # -- Dir 2
			self.data_sendCan.id = 1538
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection_set[1] # 64h - 100
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 131

	 # ------------------ Protocol Write Speed Loop ------------------ #
		elif (self.sort_send == 131): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			# print ("valueSpeed_set: ", self.valueSpeed_set)

			byte_arr = self.int_to_4bytes(self.valueSpeed_set[0])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			# print ("self.data_sendCan.byte4: ", self.data_sendCan.byte4)
			self.sort_send = 132

		elif (self.sort_send == 132): # -- ID 3
			self.data_sendCan.id = 1539 # -- 
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(self.valueSpeed_set[2])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 133

		elif (self.sort_send == 133): # -- ID 4
			self.data_sendCan.id = 1540 # -- 
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(self.valueSpeed_set[3])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 134

		elif (self.sort_send == 134): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(self.valueSpeed_set[1])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 111
	
	 # ------------------ Send CAN ------------------ #
		# print ("self.sort_send: ", self.sort_send)
		if (self.sort_send < 200):
			self.pub_sendCAN.publish(self.data_sendCan)

	def sendCAN_CHECK(self):
	 # ------------------ Start ------------------ #
		if (self.sort_send == 1): # -- Reset all nodes
			print ("-- 1 --")
			self.data_sendCan.id = 0
			self.data_sendCan.byte0 = 129
			self.data_sendCan.byte1 = 0
			self.data_sendCan.byte2 = 0
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 0
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 2

		elif (self.sort_send == 2): # -- Start remote control
			print ("-- 2 --")
			self.data_sendCan.id = 0
			self.data_sendCan.byte0 = 1
			self.data_sendCan.byte1 = 0
			self.data_sendCan.byte2 = 0
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 0
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 31

	 # ------------------ Direction ------------------ #
		elif (self.sort_send == 31): # -- Dir 1
			self.data_sendCan.id = 1537
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection[0] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 32

		elif (self.sort_send == 32): # -- Dir 2
			self.data_sendCan.id = 1538
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection[1] # 64h - 100
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 33

		elif (self.sort_send == 33): # -- Dir 3
			self.data_sendCan.id = 1539
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection[2] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 34

		elif (self.sort_send == 34): # -- Dir 4
			self.data_sendCan.id = 1540
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection[3] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 41
		
	 # ------------------ Switch On 6h ------------------ #
		elif (self.sort_send == 41): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 6 # 6
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 42

		elif (self.sort_send == 42): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 6 # 6
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 43

		elif (self.sort_send == 43): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 6 # 6
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 44

		elif (self.sort_send == 44): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 6 # 6
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 51

	 # ------------------ Switch On 7h ------------------ #
		elif (self.sort_send == 51): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 7 # 7
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 52

		elif (self.sort_send == 52): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 7 # 7
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 53

		elif (self.sort_send == 53): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 7 # 7
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 54

		elif (self.sort_send == 54): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 7 # 7
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 61

	 # ------------------ Enable Driver Fh ------------------ #
		elif (self.sort_send == 61): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 15 # Fh
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 62

		elif (self.sort_send == 62): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 15 # Fh
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 63

		elif (self.sort_send == 63): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 15 # Fh
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 64

		elif (self.sort_send == 64): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 15 # Fh
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 71

	 # ------------------ Write operation mode ------------------ #
		elif (self.sort_send == 71): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 47
			self.data_sendCan.byte1 = 96
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 3
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 72

		elif (self.sort_send == 72): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 47
			self.data_sendCan.byte1 = 96
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 3
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 73

		elif (self.sort_send == 73): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 47
			self.data_sendCan.byte1 = 96
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 3
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 74

		elif (self.sort_send == 74): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 47
			self.data_sendCan.byte1 = 96
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 3
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 81

	 # ------------------ Protocol Acceleration ------------------ #
		elif (self.sort_send == 81): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 131
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(50000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 82

		elif (self.sort_send == 82): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 131
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(50000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 83

		elif (self.sort_send == 83): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 131
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(50000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 84

		elif (self.sort_send == 84): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 131
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(50000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 91

	 # ------------------ Protocol Deceleration ------------------ #
		elif (self.sort_send == 91): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 132
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(500000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 92

		elif (self.sort_send == 92): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 132
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(500000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 93

		elif (self.sort_send == 93): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 132
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(500000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 94

		elif (self.sort_send == 94): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 132
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(500000)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 101

	 # ------------------ Protocol Write Speed ------------------ #
		elif (self.sort_send == 101): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0

			byte_arr = self.int_to_4bytes(self.valueSpeed[0])
			# print ("valueSpeed: ", self.valueSpeed)
			# print ("byte_arr: ", byte_arr)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 102

		elif (self.sort_send == 102): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes( self.valueSpeed[1])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 103

		elif (self.sort_send == 103): # -- 
			self.data_sendCan.id = 1539 # -- ID 3
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(self.valueSpeed[2])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 104

		elif (self.sort_send == 104): # -- 
			self.data_sendCan.id = 1540 # -- ID 4
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(self.valueSpeed[3])
			# print ("byte_arr: ", byte_arr)
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 111

	 # ------------------ Protocol Read Speed ------------------ #
		elif (self.sort_send == 111): # -- P28
			self.data_sendCan.id = 1537
			self.data_sendCan.byte0 = 64 	# 40h - 64
			self.data_sendCan.byte1 = 108 	# 6Ch
			self.data_sendCan.byte2 = 96  	# 60h
			self.data_sendCan.byte3 = 0   	# 
			self.data_sendCan.byte4 = 0 	# 
			self.data_sendCan.byte5 = 0 	# 
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 112

		elif (self.sort_send == 112): # -- 
			self.data_sendCan.id = 1538
			self.data_sendCan.byte0 = 64 	# 40
			self.data_sendCan.byte1 = 108 	# 6Ch
			self.data_sendCan.byte2 = 96  	# 60h
			self.data_sendCan.byte3 = 0   	# 
			self.data_sendCan.byte4 = 0 	# 
			self.data_sendCan.byte5 = 0 	# 
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 113

		elif (self.sort_send == 113): # -- 
			self.data_sendCan.id = 1539
			self.data_sendCan.byte0 = 64 	# 40
			self.data_sendCan.byte1 = 108 	# 6Ch
			self.data_sendCan.byte2 = 96  	# 60h
			self.data_sendCan.byte3 = 0   	# 
			self.data_sendCan.byte4 = 0 	# 
			self.data_sendCan.byte5 = 0 	# 
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 114

		elif (self.sort_send == 114): # -- 
			self.data_sendCan.id = 1540
			self.data_sendCan.byte0 = 64 	# 40
			self.data_sendCan.byte1 = 108 	# 6Ch
			self.data_sendCan.byte2 = 96  	# 60h
			self.data_sendCan.byte3 = 0   	# 
			self.data_sendCan.byte4 = 0 	# 
			self.data_sendCan.byte5 = 0 	# 
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 121

	 # ------------------ Protocol Dir Loop ------------------ #
		elif (self.sort_send == 121): # -- Dir 1
			for i in range(4):
				self.valueDirection_set[i] = self.valueDirection[i]
				self.valueSpeed_set[i] = self.valueSpeed[i]

			self.data_sendCan.id = 1537
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection_set[0] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 122

		elif (self.sort_send == 122): # -- Dir 3
			self.data_sendCan.id = 1539
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection_set[2] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 123

		elif (self.sort_send == 123): # -- Dir 4
			self.data_sendCan.id = 1540
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection_set[3] # 64h
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 124

		elif (self.sort_send == 124): # -- Dir 2
			self.data_sendCan.id = 1538
			self.data_sendCan.byte0 = 47 # 2Fh
			self.data_sendCan.byte1 = 126
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = self.valueDirection_set[1] # 64h - 100
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 131

	 # ------------------ Protocol Write Speed Loop ------------------ #
		elif (self.sort_send == 131): # -- 
			self.data_sendCan.id = 1537 # -- ID 1
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0

			byte_arr = self.int_to_4bytes(self.valueSpeed_set[0])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 132

		elif (self.sort_send == 132): # -- ID 3
			self.data_sendCan.id = 1539 # -- 
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(self.valueSpeed_set[2])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 133

		elif (self.sort_send == 133): # -- ID 4
			self.data_sendCan.id = 1540 # -- 
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(self.valueSpeed_set[3])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 134

		elif (self.sort_send == 134): # -- 
			self.data_sendCan.id = 1538 # -- ID 2
			self.data_sendCan.byte0 = 35
			self.data_sendCan.byte1 = 255
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			byte_arr = self.int_to_4bytes(self.valueSpeed_set[1])
			self.data_sendCan.byte4 = byte_arr[0]
			self.data_sendCan.byte5 = byte_arr[1]
			self.data_sendCan.byte6 = byte_arr[2]
			self.data_sendCan.byte7 = byte_arr[3]
			self.sort_send = 111
	
	 # ------------------ Send CAN ------------------ #
		# print ("self.sort_send: ", self.sort_send)
		if (self.sort_send < 200):
			self.pub_sendCAN.publish(self.data_sendCan)


	def readSpeed(self):
		if (self.data_receivedCAN.idSend == 129 and self.data_receivedCAN.byte1 == 108 and self.data_receivedCAN.byte2 == 96):
			by0 = self.data_receivedCAN.byte4
			by1 = self.data_receivedCAN.byte5
			by2 = self.data_receivedCAN.byte6
			by3 = self.data_receivedCAN.byte7
			# print ("read speed ID1: ", self.convert_4byte_int(by0, by1, by2, by3) )
			self.driver_respond.speed1 = self.convert_4byte_int(by0, by1, by2, by3)

		if (self.data_receivedCAN.idSend == 130 and self.data_receivedCAN.byte1 == 108 and self.data_receivedCAN.byte2 == 96):
			by0 = self.data_receivedCAN.byte4
			by1 = self.data_receivedCAN.byte5
			by2 = self.data_receivedCAN.byte6
			by3 = self.data_receivedCAN.byte7
			# print ("read speed ID2: ", self.convert_4byte_int(by0, by1, by2, by3) )
			self.driver_respond.speed2 = self.convert_4byte_int(by0, by1, by2, by3)

		if (self.data_receivedCAN.idSend == 131 and self.data_receivedCAN.byte1 == 108 and self.data_receivedCAN.byte2 == 96):
			by0 = self.data_receivedCAN.byte4
			by1 = self.data_receivedCAN.byte5
			by2 = self.data_receivedCAN.byte6
			by3 = self.data_receivedCAN.byte7
			# print ("read speed ID3: ", self.convert_4byte_int(by0, by1, by2, by3) )
			self.driver_respond.speed3 = self.convert_4byte_int(by0, by1, by2, by3)

		if (self.data_receivedCAN.idSend == 132 and self.data_receivedCAN.byte1 == 108 and self.data_receivedCAN.byte2 == 96):
			by0 = self.data_receivedCAN.byte4
			by1 = self.data_receivedCAN.byte5
			by2 = self.data_receivedCAN.byte6
			by3 = self.data_receivedCAN.byte7
			# print ("read speed ID4: ", self.convert_4byte_int(by0, by1, by2, by3) )
			self.driver_respond.speed4 = self.convert_4byte_int(by0, by1, by2, by3)

			print ("--------------------------------")

	def check_frame(self):
		self.data_sendCan.id = 1536 + 4
		self.data_sendCan.byte0 = 64 	# 40
		self.data_sendCan.byte1 = 108 	# 6Ch
		self.data_sendCan.byte2 = 96  	# 60h
		self.data_sendCan.byte3 = 0   	# 
		self.data_sendCan.byte4 = 0 	# 
		self.data_sendCan.byte5 = 0 	# 
		self.data_sendCan.byte6 = 0
		self.data_sendCan.byte7 = 0
		self.sort_send = 2

		self.pub_sendCAN.publish(self.data_sendCan)

	def checkFrame_readSpeed(self, ID):
		self.data_sendCan.id = 1536 + ID
		self.data_sendCan.byte0 = 64 	# 40
		self.data_sendCan.byte1 = 108 	# 6Ch
		self.data_sendCan.byte2 = 96  	# 60h
		self.data_sendCan.byte3 = 0   	# 
		self.data_sendCan.byte4 = 0 	# 
		self.data_sendCan.byte5 = 0 	# 
		self.data_sendCan.byte6 = 0
		self.data_sendCan.byte7 = 0
		self.pub_sendCAN.publish(self.data_sendCan)

	def checkFrame_enableServo(self, ID):
		if (self.sort_send == 1): # -- Reset all nodes
			# self.data_sendCan.id = 0
			# self.data_sendCan.byte0 = 129 # 81
			# self.data_sendCan.byte1 = ID # 0
			# self.data_sendCan.byte2 = 0
			# self.data_sendCan.byte3 = 0
			# self.data_sendCan.byte4 = 0
			# self.data_sendCan.byte5 = 0
			# self.data_sendCan.byte6 = 0
			# self.data_sendCan.byte7 = 0
			self.sort_send = 2

		elif (self.sort_send == 2): # -- 
			self.data_sendCan.id = 0
			self.data_sendCan.byte0 = 1
			self.data_sendCan.byte1 = ID
			self.data_sendCan.byte2 = 0
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 0
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 5

		elif (self.sort_send == 3): # -- 
			self.data_sendCan.id = 1536 + ID # 1540
			self.data_sendCan.byte0 = 43 # 2B
			self.data_sendCan.byte1 = 64 # 40
			self.data_sendCan.byte2 = 96 # 60
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 6 # 6
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 7

		elif (self.sort_send == 4): # -- 
			self.data_sendCan.id = 1536 + ID # 1540
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 7 # 7
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 5

		elif (self.sort_send == 5): # -- 
			self.data_sendCan.id = 1536 + ID # 1540
			self.data_sendCan.byte0 = 43
			self.data_sendCan.byte1 = 64
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 15 # 
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 6

		elif (self.sort_send == 6): # -- 
			self.data_sendCan.id = 1536 + ID # 1540
			self.data_sendCan.byte0 = 47
			self.data_sendCan.byte1 = 96
			self.data_sendCan.byte2 = 96
			self.data_sendCan.byte3 = 0
			self.data_sendCan.byte4 = 3
			self.data_sendCan.byte5 = 0
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			self.sort_send = 7

		elif (self.sort_send == 7): # -- protocol speed
			self.data_sendCan.id = 1536 + ID
			self.data_sendCan.byte0 = 64 	# 40
			self.data_sendCan.byte1 = 108 	# 6Ch
			self.data_sendCan.byte2 = 96  	# 60h
			self.data_sendCan.byte3 = 0   	# 
			self.data_sendCan.byte4 = 0 	# 
			self.data_sendCan.byte5 = 0 	# 
			self.data_sendCan.byte6 = 0
			self.data_sendCan.byte7 = 0
			# self.sort_send = 122

		self.pub_sendCAN.publish(self.data_sendCan)

	def receivedCAN_run(self):
		self.readSpeed()
		# pass

	def writeRead_check(self, id_device, COB, data_array):
		# -- Write
		self.data_sendCan.id = id_device + COB
		self.data_sendCan.byte0 = data_array[0]
		self.data_sendCan.byte1 = data_array[1]
		self.data_sendCan.byte2 = data_array[2]
		self.data_sendCan.byte3 = data_array[3]
		self.data_sendCan.byte4 = data_array[4]
		self.data_sendCan.byte5 = data_array[5]
		self.data_sendCan.byte6 = data_array[6]
		self.data_sendCan.byte7 = data_array[7]

		# - Read Check
		if self.data_receivedCAN.idSend == (128 + id_device):
			if self.data_receivedCAN.byte0 == self.data_sendCan.byte0 and self.data_receivedCAN.byte1 == self.data_sendCan.byte1 and self.data_receivedCAN.byte2 == self.data_sendCan.byte2 and self.data_receivedCAN.byte2 == self.data_sendCan.byte2:
				return 1
		return 0

	def string_arry(self):
		arr_str = "0123456"
		print ("OUT0: ", arr_str[2:4])
		print ("OUT1: ", arr_str[0:2])
		print ("OUT2: ", arr_str[:2])
		print ("OUT3: ", arr_str[:-3])
		print ("OUT4: ", arr_str[-2:0])
		print ("OUT5: ", arr_str[2:0])

	def run(self):
		while not rospy.is_shutdown():
			# -- SEND CAN
			delta_time = (time.time() - self.saveTime_sendCAN)%60 
			if (delta_time > 1/self.frequence_sendCAN):
				self.saveTime_sendCAN = time.time()

				self.sendCAN_All()
				self.pub_driverRespond.publish(self.driver_respond)

			# self.checkFrame_enableServo(4)
			self.rate.sleep()

def main():
	print('Program starting')
	program = CAN_ROS()
	program.run()
	# program.try_run()
	# program.string_arry()
	
	print('Programer stopped')

if __name__ == '__main__':
    main()

"""
Quy trình giao tiếp:
1, 
"""
