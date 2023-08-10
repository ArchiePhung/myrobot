#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Authors : BEE
# kinematic.
# DATE: 20/07/2022
# AUTHOR: HOANG VAN QUANG

import rospy
import sys
import time
import threading
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from message_pkg.msg import Driver_query, Mecanum_respond, Mecanum_request
from message_pkg.msg import Driver_respond

# from sti_msgs.msg import Mecanum_respond, Mecanum_request
from std_msgs.msg import Int16

from math import sin , cos , pi , atan2

class kinematic(threading.Thread):
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('kinematic', anonymous= False) # False
		

		# -- parameter
		self.wheel_circumference = rospy.get_param("wheel_circumference", 0.4787) # - 6 inchs - 0.1524 m
		self.transmission_ratio  = rospy.get_param("transmission_ratio", 20.0)
		self.wheels_x_distance = rospy.get_param("wheels_x_distance", 0.72)
		self.wheels_y_distance = rospy.get_param("wheels_y_distance", 0.60675)
		self.SPR = rospy.get_param("step_per_round", 10000) # -- Step Per Round.

		self.frequency_control = rospy.get_param("frequency_control", 25.0)
		self.linear_max  = rospy.get_param("linear_max", 0.4) # m/s
		self.angular_max = rospy.get_param("angular_max", 0.34) # rad/s
		self.max_rpm = rospy.get_param("max_rpm", 3800)
		
		self.topicControl_vel = rospy.get_param("topicControl_vel", "/cmd_vel") # 
		self.topicGet_vel = rospy.get_param("topicGet_vel", "/raw_vel") # 

		self.topicControl_driver = rospy.get_param("topicControl_driver", "/mecanum_request")
		self.topicRespond_driver = rospy.get_param("topicRespond_driver", "/mecanum_respond")

		self.frame_id = rospy.get_param("frame_id", "frame_robot")
		# --	
		self.rate = rospy.Rate(self.frequency_control)

		rospy.Subscriber(self.topicControl_vel, Twist, self.cmdVel_callback)
		self.cmd_vel = Twist()

		self.pub_rawVel = rospy.Publisher(self.topicGet_vel, Twist, queue_size=50)
		self.raw_vel = Twist()

		# -
		rospy.Subscriber(self.topicRespond_driver, Mecanum_respond, self.driver_callback)
		self.driver_respond = Mecanum_respond()

		self.pub_driver = rospy.Publisher(self.topicControl_driver, Mecanum_request, queue_size=50)
		self.driver_query = Mecanum_request()
		# -- 
		self.nowTime_cmdVel = time.time()
		self.isTimeout_cmdVel = 0
		# -- 
		self.nowTime_resspondDriver = time.time()
		self.isTimeout_resspondDriver = 0

	def cmdVel_callback(self, data):
		self.cmd_vel = data

	def driver_callback(self, data):
		self.driver_respond = data

	def constrain(self, value_in, value_min, value_max):
		value_out = 0.0
		if (value_in < value_min):
			value_out = value_min
		elif (value_in > value_max):
			value_out = value_max
		else:
			value_out = value_in
		return value_out

	def calculate_speed(self):
		linear_x = 0.0
		linear_y = 0.0
		angular_z = 0.0
		# -- Gioi han.
		linear_x = self.constrain(self.cmd_vel.linear.x, -self.linear_max, self.linear_max)
		# -
		linear_y = self.constrain(self.cmd_vel.linear.y, -self.linear_max, self.linear_max)
		# - 
		angular_z = self.constrain(self.cmd_vel.angular.z, -self.angular_max, self.angular_max)
		# --
		# -- convert m/s to m/min
		linear_vel_x_mins = linear_x * 60
		linear_vel_y_mins = linear_y * 60

		# -- convert rad/s to rad/min
		angular_vel_z_mins = angular_z * 60
		# --  
		tangential_vel = angular_vel_z_mins*(self.wheels_x_distance/2. + self.wheels_y_distance/2.)
		# - 
		x_rpm = linear_vel_x_mins/self.wheel_circumference
		y_rpm = linear_vel_y_mins/self.wheel_circumference
		tan_rpm = tangential_vel/self.wheel_circumference

		# # -- front-left motor
		# RPM1 = x_rpm - y_rpm - tan_rpm
		# # -- front-right motor
		# RPM2 = x_rpm + y_rpm + tan_rpm
		# # -- rear-left motor
		# RPM3 = x_rpm + y_rpm - tan_rpm
		# # -- rear-right motor
		# RPM4 = x_rpm - y_rpm + tan_rpm

		# -- front-left motor
		RPM1 = x_rpm - y_rpm - tan_rpm
		# -- front-right motor
		RPM2 = x_rpm + y_rpm + tan_rpm
		# -- rear-left motor
		RPM3 = x_rpm + y_rpm - tan_rpm
		# -- rear-right motor
		RPM4 = x_rpm - y_rpm + tan_rpm

		self.driver_query.speed1 = int( (RPM1/60.)*self.transmission_ratio*self.SPR )
		self.driver_query.speed2 = int( (RPM2/60.)*self.transmission_ratio*self.SPR )
		self.driver_query.speed3 = int( (RPM3/60.)*self.transmission_ratio*self.SPR )
		self.driver_query.speed4 = int( (RPM4/60.)*self.transmission_ratio*self.SPR )

	def get_speed(self):
		average_rps_x = 0.0
		average_rps_y = 0.0
		average_rps_a = 0.0

		rpm1 = (self.driver_respond.speed1*60)/self.transmission_ratio/self.SPR
		rpm2 = (self.driver_respond.speed2*60)/self.transmission_ratio/self.SPR
		rpm3 = (self.driver_respond.speed3*60)/self.transmission_ratio/self.SPR
		rpm4 = (self.driver_respond.speed4*60)/self.transmission_ratio/self.SPR

		# -- Convert average revolutions per minute to revolutions per second
		average_rps_x = ( (rpm1 + rpm2 + rpm3 + rpm4) / 4.) / 60. # - RPM
		self.raw_vel.linear.x = average_rps_x * self.wheel_circumference # - m/s

		# -- Convert average revolutions per minute in y axis to revolutions per second
		average_rps_y = ( (-rpm1 + rpm2 + rpm3 - rpm4) / 4.) / 60. # - RPM
		self.raw_vel.linear.y = average_rps_y * self.wheel_circumference # - m/s

		# -- Convert average revolutions per minute to revolutions per second
		average_rps_a = ( (-rpm1 + rpm2 - rpm3 + rpm4) / 4.) / 60.
		self.raw_vel.angular.z = (average_rps_a * self.wheel_circumference) / ((self.wheels_x_distance/ 2.) + (self.wheels_y_distance/ 2.)) # - rad/s

	def run(self):
		while not rospy.is_shutdown():
			# -
			self.calculate_speed()
			# -
			self.get_speed()
			# -
			self.pub_driver.publish(self.driver_query)
			# -
			self.pub_rawVel.publish(self.raw_vel)
			# -
			self.rate.sleep()
def main():
	print('Starting Program')
	programer = kinematic()
	programer.run()
	print('Exiting Program')

if __name__ == '__main__':
	main()
