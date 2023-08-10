#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Authors : BEE
# DATE: 15/07/2022
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal

from geometry_msgs.msg import Twist, Pose, Point

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from math import sin, cos, pi, atan2, radians, sqrt, pow, degrees, fabs

"""
IN: frame_Robot follow frame_Goal
OUT: frame_Goal follow frame_Robot
"""
# def convertCoordinate_robot_goal(self, poseRobot, poseGoal): # -- Pose.
# 	distancePoint = 

def calculateAngle_point(p1, p2): # p1, p2 | geometry_msgs/Point
	d_x = p2.x - p1.x
	d_y = p2.y - p1.y
	ang = 0
	if (d_x == 0):
		if (d_y >= 0):
			ang = pi/2.
		else:
			ang = -pi/2.
	else:
		if (d_y == 0):
			if (d_x > 0):
				ang = 0
			else:
				ang = pi
		else:
			ang = atan2(d_y, d_x)
			if (ang > pi):
				ang = ang - 2*pi
			if (ang < -pi):
				ang = 2*pi + ang

	return ang 

def int_to_binary8bit(int_value):
	value_now = int_value
	arr_bit = [0, 0, 0, 0, 0, 0, 0, 0]

	if (int_value > 255):
		print ("int_to_binary8bit Error Max: ", int_value)
		arr_bit = [1, 1, 1, 1, 1, 1, 1, 1]
	elif (int_value < 0):
		print ("int_to_binary8bit Error Min: ", int_value)
		arr_bit = [0, 0, 0, 0, 0, 0, 0, 0]
	else:
	    if value_now > 2**7:
	        arr_bit[0] = 1
	        value_now = value_now - (2**7)
			
	    if value_now > 2**6:
	        arr_bit[1] = 1
	        value_now = value_now - (2**6)

	    if value_now > 2**5:
	        arr_bit[2] = 1
	        value_now = value_now - (2**5)

	    if value_now > 2**4:
	        arr_bit[3] = 1
	        value_now = value_now - (2**4)

	    if value_now > 2**3:
	        arr_bit[4] = 1
	        value_now = value_now - (2**3)

	    if value_now > 2**2:
	        arr_bit[5] = 1
	        value_now = value_now - (2**2)

	    if value_now > 2**1:
	        arr_bit[6] = 1
	        value_now = value_now - (2**1)

	    if value_now >= 2**0:
	        arr_bit[7] = 1
	        value_now = value_now - (2**0)
	return arr_bit	

def bitArray_toInt(arrBit):
	value_out = 0
	leng = len(arrBit)
	for i in range(leng):
		if (arrBit[leng - i - 1] == 1):
			value_out += 2**i
		print ("b: ", leng - i - 1)
	return value_out

def controlWord(Halt, Fault_reset, Mode_specific, Enable_operation, Quick_stop, Enable_voltage, Switch_on):
	bit_arr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	# - bit No.8
	bit_arr[7] = Halt
	# - bit No.7
	bit_arr[8] = Fault_reset
	# - bit No.6~4
	bits = int_to_binary8bit(Mode_specific)
	bit_arr[9] = bits[2]
	bit_arr[10] = bits[1]
	bit_arr[11] = bits[0]
	# - bit No.3
	bit_arr[12] = Enable_operation
	# - bit No.2
	bit_arr[13] = Quick_stop
	# - bit No.1
	bit_arr[14] = Enable_voltage
	# - bit No.0
	bit_arr[15] = Switch_on

	return bitArray_toInt(bit_arr)
	
def int_to_4bytes(value): # => STT: [byte No.0, byte No.1, byte No.2, byte No.3]
	value_now = 0
	if (value >= 0):
		value_now = value
	else:
		value_now = (256**4) + value

	byteArray = [0, 0, 0, 0]
	byteArray[3] =  value_now/(256**3)
	byteArray[2] = (value_now - byteArray[3]*(256**3))/256**2
	byteArray[1] = (value_now - byteArray[3]*(256**3) - byteArray[2]*(256**2) )/256
	byteArray[0] =  value_now - byteArray[3]*(256**3) - byteArray[2]*(256**2) - byteArray[1]*256
	return byteArray


p1 = Point()
p2 = Point()

p1.x = 1
p1.y = 1

p2.x = 4
p2.y = -1
# ang = calculateAngle_point(p1, p2)

# print ("ang: ", degrees(ang) )
# print ("val: ", abs(-5.3) )

# bits = int_to_binary8bit(65)
# print ("bits: ", bits)

# val = bitArray_toInt([1, 0, 1, 1, 1])
# print ("val: ", val)

# value_control = controlWord(0, 0, 0, 0, 1, 1, 0)
# print ("value_control: ", value_control)


print ("int_to_4bytes: ", int_to_4bytes(-250000))