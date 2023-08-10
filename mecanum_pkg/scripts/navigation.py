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

from geometry_msgs.msg import Twist, Pose, Point, Quaternion

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sti_msgs.msg import Velocities

from math import sin, cos, pi, atan2, radians, sqrt, pow, degrees
"""
Di chuyển theo đường dẫn tiêu chuẩn.

1, Lấy Pose của điểm Goal tương đối trên đường dẫn.
2, Tính toán độ lệch về khoảng cách và góc.
3, Tính toán tốc độ yêu cầu.
"""
class testRun():
	def __init__(self):
		# ---------------------------------------- ROS
		print("ROS Initial!")
		rospy.init_node('testRun', anonymous=False)
		self.rate = rospy.Rate(24)

		self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
		self.cmd_vel = Twist()

		rospy.Subscriber('/odom', Odometry, self.odometry_callback, queue_size = 20)
		self.odom_robot = Odometry()
		self.poseRobot = Pose()
		self.isReceived_poseRobot = 0

		rospy.Subscriber('/path_move', Path, self.path_callback, queue_size = 20)
		self.path_move = Path()
		# --
		self.goalNow = Pose()
		self.delta_x = 0
		self.delta_y = 0
		self.delta_anglePoint = 0
		self.delta_angleTarget = 0
		# -- 
		self.step_Goal = 0
		self.accuracyDistane_level1 = 0.04 # - m
		self.accuracyDistane_level2 = 0.004 # - m
		self.accuracyDistane = self.accuracyDistane_level1 #

		self.accuracyAngle_level1 = radians(4) # - rad
		self.accuracyAngle_level2 =  radians(1) # - rad
		self.accuracyAngle = self.accuracyAngle_level1 # 

		self.deceleration_corner = radians(15) # -- rad - Ngưỡng góc giảm tốc độ quay.
		self.deceleration_distance = 0.6 # -- m - Ngưỡng khoảng cách giảm tốc độ quay.

		self.max_linear = 0.5 # -- m/s - Tốc độ tịnh tiến tối đa.
		self.max_angular = 0.3 # -- m/s - Tốc độ tịnh tiến tối đa.

		self.min_linear = 0.02 # -- m/s - Tốc độ tịnh tiến tối đa.
		self.min_angular = 0.04 # -- m/s - Tốc độ tịnh tiến tối đa.

		# --
		self.complete_distance = 0
		self.complete_anglePoint = 0
		self.complete_angleTarget = 0 
		self.complete_Goal = 0
		# -- 
		self.velocities_lv1 = Velocities()
		self.velocities_lv2 = Velocities()
		# -- 
		self.poseFake_robot = Pose()
		self.poseFake_goal = Pose()
		# -- 
		self.axis_x = 1
		self.axis_y = 2
		self.mainFollow_axis = self.axis_x
		self.rateDelta_x_y = 0.0 # -- X/Y
		self.rateVel_x_y = 0.0 # -- X/Y

		# self.velocities_lv1.angular_z = 0.3
		# -- PID Corner
		self.Kp = 0.0
		self.Ki = 0.0
		self.Kd = 0.0
		self.Error = 0.0
		self.Pre_Error = 0.0
		self.value_I = 0.0
		self.value_PID = 0.0
		self.preTime = time.time()
		
	def odometry_callback(self, data):
		self.odom_robot = data
		self.poseRobot = self.odom_robot.pose.pose
		self.isReceived_poseRobot = 1
		# print ("REC -- ")

	def path_callback(self, data):
		self.path_move = data

	# -------------------
	def constrain(self, value_in, value_min, value_max):
		value_out = 0.0
		if (value_in < value_min):
			value_out = value_min
		elif (value_in > value_max):
			value_out = value_max
		else:
			value_out = value_in
		return value_out

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

	def calculate_distance(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return sqrt(x*x + y*y)

	def calculateDelta_XY(self, p1, p2): # p1, p2 | geometry_msgs/Point
		x = p2.x - p1.x
		y = p2.y - p1.y
		return x, y

	def calculateAngle_qua(self, qua1, qua2): # q1, q2 | geometry_msgs/Quaternion
		euler1 = self.quaternion_to_euler(qua1)
		euler2 = self.quaternion_to_euler(qua2)

		delta_angle = euler2 - euler1
		if (abs(delta_angle) >= pi):
			if (delta_angle >= 0):
				delta_angle = (pi*2 - abs(delta_angle))*(-1)
			else:
				delta_angle = pi*2 - abs(delta_angle)
		return delta_angle

	def calculateAngle_point(self, p1, p2): # p1 to p2 | geometry_msgs/Point
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

	def calculateAngle_robotToPoint(self, poseRobot, poseGoal): # robot to goal | geometry_msgs/Point
		del_point = self.calculateAngle_point(poseRobot.position, poseGoal.position)
		del_robot = self.quaternion_to_euler(poseRobot.orientation)
		delta_angle = del_point - del_robot

		if abs(delta_angle) >= pi:
			ang = (2*pi - abs(delta_angle))
			if angle_fn > 0:
				delta_angle = -ang
			else:
				delta_angle = ang
		return delta_angle

	def convertCoordinate_robot_goal(self, poseRobot, poseGoal): # -- Pose.
		distancePoint = self.calculate_distance(poseRobot.position, poseGoal.position)

		angle_goalToRobot = self.calculateAngle_point(poseGoal.position, poseRobot.position)
		angle_robotToGoal = self.calculateAngle_point(poseRobot.position, poseGoal.position)
		
		angle_goal = self.quaternion_to_euler(poseGoal.orientation)
		angle_robot = self.quaternion_to_euler(poseRobot.orientation)
		
		# print ("angle_robot: ", degrees(angle_robot))
		# print ("angle_goal: ", degrees(angle_goal))
		# print ("angle_robotToGoal: ", degrees(angle_robotToGoal))
		

		# -- 
		delta_angle = angle_robotToGoal - angle_robot
		if (abs(delta_angle) > pi):
			if (delta_angle > 0):
				delta_angle = delta_angle - 2*pi

			if (delta_angle < 0):
				delta_angle = delta_angle + 2*pi
		# print ("delta_angle: ", degrees(delta_angle) )

		# -- 
		deltaAngle_goal_robot = angle_goal - angle_robot
		# print ("deltaAngle_goal_robot1: ", degrees(deltaAngle_goal_robot) )
		if (abs(deltaAngle_goal_robot) > pi):
			if (deltaAngle_goal_robot > 0):
				deltaAngle_goal_robot = deltaAngle_goal_robot - 2*pi

			if (deltaAngle_goal_robot < 0):
				deltaAngle_goal_robot = deltaAngle_goal_robot + 2*pi
		# print ("deltaAngle_goal_robot2: ", degrees(deltaAngle_goal_robot) )

		# -- 
		poseGoal_new = Pose()
		poseGoal_new.position.x = cos(delta_angle)*distancePoint
		poseGoal_new.position.y = sin(delta_angle)*distancePoint

		# print ("poseGoal_new X: ", poseGoal_new.position.x)
		# print ("poseGoal_new Y: ", poseGoal_new.position.y)

		# print (str(round(poseGoal_new.position.x, 3)) + " | " + str(round(poseGoal_new.position.y, 3)))

		poseRobot_new = Pose()
		poseRobot_new.orientation = self.euler_to_quaternion(0)

		return poseRobot_new, poseGoal_new
	# -------------------
	def folow_path(self):
		pass

	""" 
		Varibale: modeDistance
			Value | mean
			0 | Độ chính xác cấp 1.
			1 | Độ chính xác cấp 2. Cao hơn.
		Varibale: modeAngle
			Value | mean
			0 | Không đáp ứng góc.
			1 | Đáp ứng góc khi hoàn thành đáp ứng khoảng cách.
			2 | Đáp ứng góc và đáp ứng khoảng cách xảy ra đồng thời.
	"""

	def keeping_corner(self, poseRobot, angleTarget, velNow):	# -- Pose() | quaternion | Twist()
		delta_time = (time.time() - self.preTime)%60
		if (delta_time >= 0.04):
			self.preTime = time.time()

			offset_angular_z = 0.0
			angle_robot = self.quaternion_to_euler(poseRobot.orientation)
			delta_angle = angleTarget - angle_robot

			self.Error = delta_angle

			if (abs(self.value_PID) <= self.max_angular):
				self.value_I += self.Error

			value_D = self.Error - self.Pre_Error


			self.value_PID = self.Kp*self.Error + self.Ki*self.value_I + self.Kd*value_D # -- Gía trị tốc độ góc.
			# -- 
			if (self.value_PID < -self.max_angular):
				self.value_PID = -self.max_angular

			if (self.value_PID > self.max_angular):
				self.value_PID = self.max_angular

		if (abs(delta_angle) > radians(30)): # -- Độ lệch góc quá lớn -> xoay -> tiếp tục di chuyển.
			pass
		else:
			if (abs(delta_angle) < radians(0.4)): # -- Sai lệch góc đủ lớn mới cho phép điều chỉnh.
				offset_angular_z = 0.0
			else:
				if (abs(delta_angle) > self.deceleration_corner):
					if (delta_angle > 0):
						offset_angular_z = self.max_angular
					else:
						offset_angular_z = -self.max_angular
				else:
					rate = delta_angle/self.deceleration_corner
					offset_angular_z = self.max_angular*rate


	def folowGoal_keepCorner(self, poseRobot, poseGoal, modeDistance, modeAngle): 
		# -- Tính toán độ lệch.
		self.delta_x, self.delta_y = self.calculateDelta_XY(poseRobot.position, poseGoal.position)
		self.delta_anglePoint = self.calculateAngle_robotToPoint(poseRobot, poseGoal)
		self.delta_angleTarget = self.calculateAngle_qua(poseRobot.orientation, poseGoal.orientation)
		self.rate_x_y = self.delta_x/self.delta_y
		# -- 
		if (modeDistance == 0):
			self.accuracyDistane = self.accuracyDistane_level1			
		else:
			self.accuracyDistane = self.accuracyDistane_level2

		# - Kiểm tra đáp ứng - Xác nhận hoàn thành.
		if (abs(self.delta_x) <= self.accuracyDistane and abs(self.delta_y) <= self.accuracyDistane):
			self.complete_distance = 1

		if (abs(self.delta_anglePoint) <= self.accuracyAngle):
			self.complete_anglePoint = 1

		if (modeAngle == 1):
			self.complete_angle = 1

		self.complete_anglePoint = 1
		if (self.step_Goal == 0): # - Đáp ứng góc ban đầu.
			self.velocities_lv1.linear_x = 0
			self.velocities_lv1.linear_y = 0
			
			if (abs(self.delta_anglePoint) <= self.deceleration_corner):
				rate = self.delta_anglePoint/self.deceleration_corner
				self.velocities_lv1.angular_z = self.max_angular*rate

				if (abs(self.velocities_lv1.angular_z) < self.min_angular):
					if (self.velocities_lv1.angular_z > 0):
						self.velocities_lv1.angular_z = self.min_angular
					else:
						self.velocities_lv1.angular_z = -self.min_angular

			else:
				if (self.delta_anglePoint > 0):
					self.velocities_lv1.angular_z = self.max_angular
				else:
					self.velocities_lv1.angular_z = -self.max_angular

			if (self.complete_anglePoint == 1):
				self.step_Goal = 1
				self.velocities_lv1.angular_z = 0

		if (self.step_Goal == 1): # - Đáp ứng khoảng cách.
			# print ("aaa")
			
			# -- Chọn trục chính.
			if (abs(self.delta_x) >= abs(self.delta_y)):
				self.mainFollow_axis = self.axis_x
			else:
				self.mainFollow_axis = self.axis_y

			if (self.mainFollow_axis == self.axis_x):
				# -- linear_x
				if (abs(self.delta_x) <= self.deceleration_distance):
					rate = self.delta_x/self.deceleration_distance
					self.velocities_lv1.linear_x = self.max_linear*rate
					if (abs(self.delta_x) <= self.accuracyDistane):
						self.velocities_lv1.linear_x = 0

					rate = self.delta_y/self.deceleration_distance
					self.velocities_lv1.linear_y = self.max_linear*rate
					if (abs(self.delta_y) <= self.accuracyDistane):
						self.velocities_lv1.linear_y = 0
						
				else:
					if (self.delta_x < 0):
						self.velocities_lv1.linear_x = -self.max_linear
					else:
						self.velocities_lv1.linear_x = self.max_linear

					if (abs(self.velocities_lv1.linear_x) < self.min_linear):
						if (self.velocities_lv1.linear_x < 0):
							self.velocities_lv1.linear_x = -self.min_linear
						else:
							self.velocities_lv1.linear_x = self.min_linear

					# -- linear_y
					# if (abs(self.delta_y) <= self.deceleration_distance):
					# 	rate = self.delta_y/self.deceleration_distance
					# 	self.velocities_lv1.linear_y = self.max_linear*rate
					# 	if (abs(self.delta_y) <= self.accuracyDistane):
					# 		self.velocities_lv1.linear_y = 0
						
					# else:
					if (self.delta_y < 0):
						self.velocities_lv1.linear_y = -abs(self.velocities_lv1.linear_x/self.rate_x_y)
					else:
						self.velocities_lv1.linear_y = abs(self.velocities_lv1.linear_x/self.rate_x_y)

					if (abs(self.velocities_lv1.linear_y) < self.min_linear):
						if (self.velocities_lv1.linear_y < 0):
							self.velocities_lv1.linear_y = -self.min_linear
						else:
							self.velocities_lv1.linear_y = self.min_linear

					if (abs(self.delta_y) <= self.accuracyDistane):
						self.velocities_lv1.linear_y = 0

			else:
				# -- linear_y
				if (abs(self.delta_y) <= self.deceleration_distance):
					rate = self.delta_y/self.deceleration_distance
					self.velocities_lv1.linear_y = self.max_linear*rate
					if (abs(self.delta_y) <= self.accuracyDistane):
						self.velocities_lv1.linear_y = 0

					rate = self.delta_x/self.deceleration_distance
					self.velocities_lv1.linear_x = self.max_linear*rate
					if (abs(self.delta_x) <= self.accuracyDistane):
						self.velocities_lv1.linear_x = 0
						
				else:
					if (self.delta_y < 0):
						self.velocities_lv1.linear_y = -self.max_linear
					else:
						self.velocities_lv1.linear_y = self.max_linear

					if (abs(self.velocities_lv1.linear_y) < self.min_linear):
						if (self.velocities_lv1.linear_y < 0):
							self.velocities_lv1.linear_y = -self.min_linear
						else:
							self.velocities_lv1.linear_y = self.min_linear

					# -- linear_x
					# if (abs(self.delta_x) <= self.deceleration_distance):
					# 	rate = self.delta_x/self.deceleration_distance
					# 	self.velocities_lv1.linear_x = self.max_linear*rate
					# 	if (abs(self.delta_x) <= self.accuracyDistane):
					# 		self.velocities_lv1.linear_x = 0
						
					# else:
					if (self.delta_x < 0):
						self.velocities_lv1.linear_x = -abs(self.velocities_lv1.linear_y*self.rate_x_y)
					else:
						self.velocities_lv1.linear_x = abs(self.velocities_lv1.linear_y*self.rate_x_y)

					if (abs(self.velocities_lv1.linear_x) < self.min_linear):
						if (self.velocities_lv1.linear_x < 0):
							self.velocities_lv1.linear_x = -self.min_linear
						else:
							self.velocities_lv1.linear_x = self.min_linear

					if (abs(self.delta_x) <= self.accuracyDistane):
						self.velocities_lv1.linear_x = 0
			# -- angular_z
			self.velocities_lv1.angular_z = 0

			if (self.complete_distance == 1):
				self.step_Goal = 2
				self.velocities_lv1.linear_x = 0
				self.velocities_lv1.linear_y = 0
				self.velocities_lv1.angular_z = 0

			print ("POS: " + str(round(self.poseRobot.position.x, 3)) + " | "  + str(round(self.poseRobot.position.y, 3)) + " | Del: " + str(round(self.delta_x, 3)) + " | "  + str(round(self.delta_y, 3)) + " |Vel| "  + str(round(self.velocities_lv1.linear_x, 3)) + " | "  + str(round(self.velocities_lv1.linear_y, 3))  )

		if (self.step_Goal == 2): # - Đáp ứng góc cuối.
			self.complete_anglePoint = 1
			if (abs(self.delta_anglePoint) <= self.accuracyAngle):
				self.complete_anglePoint = 1

		# -- Tính toán tốc độ.

	def folowGoal_rotation(self, poseRobot, poseGoal, modeDistance, modeAngle): 
		# -- Tính toán độ lệch.
		self.delta_x, self.delta_y = self.calculateDelta_XY(poseRobot.position, poseGoal.position)
		self.delta_anglePoint = self.calculateAngle_robotToPoint(poseRobot, poseGoal)
		self.delta_angleTarget = self.calculateAngle_qua(poseRobot.orientation, poseGoal.orientation)
		self.rate_x_y = self.delta_x/self.delta_y
		# -- 
		if (modeDistance == 0):
			self.accuracyDistane = self.accuracyDistane_level1			
		else:
			self.accuracyDistane = self.accuracyDistane_level2

		# - Kiểm tra đáp ứng - Xác nhận hoàn thành.
		if (abs(self.delta_x) <= self.accuracyDistane and abs(self.delta_y) <= self.accuracyDistane):
			self.complete_distance = 1

		if (abs(self.delta_anglePoint) <= self.accuracyAngle):
			self.complete_anglePoint = 1

		if (modeAngle == 1):
			self.complete_angle = 1

		self.complete_anglePoint = 1
		if (self.step_Goal == 0): # - Đáp ứng góc ban đầu.
			self.velocities_lv1.linear_x = 0
			self.velocities_lv1.linear_y = 0
			
			if (abs(self.delta_anglePoint) <= self.deceleration_corner):
				rate = self.delta_anglePoint/self.deceleration_corner
				self.velocities_lv1.angular_z = self.max_angular*rate

				if (abs(self.velocities_lv1.angular_z) < self.min_angular):
					if (self.velocities_lv1.angular_z > 0):
						self.velocities_lv1.angular_z = self.min_angular
					else:
						self.velocities_lv1.angular_z = -self.min_angular

			else:
				if (self.delta_anglePoint > 0):
					self.velocities_lv1.angular_z = self.max_angular
				else:
					self.velocities_lv1.angular_z = -self.max_angular

			if (self.complete_anglePoint == 1):
				self.step_Goal = 1
				self.velocities_lv1.angular_z = 0

		if (self.step_Goal == 1): # - Đáp ứng khoảng cách.
			# print ("aaa")
			
			# -- Chọn trục chính.
			if (abs(self.delta_x) >= abs(self.delta_y)):
				self.mainFollow_axis = self.axis_x
			else:
				self.mainFollow_axis = self.axis_y

			if (self.mainFollow_axis == self.axis_x):
				# -- linear_x
				if (abs(self.delta_x) <= self.deceleration_distance):
					rate = self.delta_x/self.deceleration_distance
					self.velocities_lv1.linear_x = self.max_linear*rate
					if (abs(self.delta_x) <= self.accuracyDistane):
						self.velocities_lv1.linear_x = 0

					rate = self.delta_y/self.deceleration_distance
					self.velocities_lv1.linear_y = self.max_linear*rate
					if (abs(self.delta_y) <= self.accuracyDistane):
						self.velocities_lv1.linear_y = 0
						
				else:
					if (self.delta_x < 0):
						self.velocities_lv1.linear_x = -self.max_linear
					else:
						self.velocities_lv1.linear_x = self.max_linear

					if (abs(self.velocities_lv1.linear_x) < self.min_linear):
						if (self.velocities_lv1.linear_x < 0):
							self.velocities_lv1.linear_x = -self.min_linear
						else:
							self.velocities_lv1.linear_x = self.min_linear

					# -- linear_y
					# if (abs(self.delta_y) <= self.deceleration_distance):
					# 	rate = self.delta_y/self.deceleration_distance
					# 	self.velocities_lv1.linear_y = self.max_linear*rate
					# 	if (abs(self.delta_y) <= self.accuracyDistane):
					# 		self.velocities_lv1.linear_y = 0
						
					# else:
					if (self.delta_y < 0):
						self.velocities_lv1.linear_y = -abs(self.velocities_lv1.linear_x/self.rate_x_y)
					else:
						self.velocities_lv1.linear_y = abs(self.velocities_lv1.linear_x/self.rate_x_y)

					if (abs(self.velocities_lv1.linear_y) < self.min_linear):
						if (self.velocities_lv1.linear_y < 0):
							self.velocities_lv1.linear_y = -self.min_linear
						else:
							self.velocities_lv1.linear_y = self.min_linear

					if (abs(self.delta_y) <= self.accuracyDistane):
						self.velocities_lv1.linear_y = 0

			else:
				# -- linear_y
				if (abs(self.delta_y) <= self.deceleration_distance):
					rate = self.delta_y/self.deceleration_distance
					self.velocities_lv1.linear_y = self.max_linear*rate
					if (abs(self.delta_y) <= self.accuracyDistane):
						self.velocities_lv1.linear_y = 0

					rate = self.delta_x/self.deceleration_distance
					self.velocities_lv1.linear_x = self.max_linear*rate
					if (abs(self.delta_x) <= self.accuracyDistane):
						self.velocities_lv1.linear_x = 0
						
				else:
					if (self.delta_y < 0):
						self.velocities_lv1.linear_y = -self.max_linear
					else:
						self.velocities_lv1.linear_y = self.max_linear

					if (abs(self.velocities_lv1.linear_y) < self.min_linear):
						if (self.velocities_lv1.linear_y < 0):
							self.velocities_lv1.linear_y = -self.min_linear
						else:
							self.velocities_lv1.linear_y = self.min_linear

					# -- linear_x
					# if (abs(self.delta_x) <= self.deceleration_distance):
					# 	rate = self.delta_x/self.deceleration_distance
					# 	self.velocities_lv1.linear_x = self.max_linear*rate
					# 	if (abs(self.delta_x) <= self.accuracyDistane):
					# 		self.velocities_lv1.linear_x = 0
						
					# else:
					if (self.delta_x < 0):
						self.velocities_lv1.linear_x = -abs(self.velocities_lv1.linear_y*self.rate_x_y)
					else:
						self.velocities_lv1.linear_x = abs(self.velocities_lv1.linear_y*self.rate_x_y)

					if (abs(self.velocities_lv1.linear_x) < self.min_linear):
						if (self.velocities_lv1.linear_x < 0):
							self.velocities_lv1.linear_x = -self.min_linear
						else:
							self.velocities_lv1.linear_x = self.min_linear

					if (abs(self.delta_x) <= self.accuracyDistane):
						self.velocities_lv1.linear_x = 0
			# -- angular_z
			self.velocities_lv1.angular_z = 0

			if (self.complete_distance == 1):
				self.step_Goal = 2
				self.velocities_lv1.linear_x = 0
				self.velocities_lv1.linear_y = 0
				self.velocities_lv1.angular_z = 0

			print ("POS: " + str(round(self.poseRobot.position.x, 3)) + " | "  + str(round(self.poseRobot.position.y, 3)) + " | Del: " + str(round(self.delta_x, 3)) + " | "  + str(round(self.delta_y, 3)) + " |Vel| "  + str(round(self.velocities_lv1.linear_x, 3)) + " | "  + str(round(self.velocities_lv1.linear_y, 3))  )

		if (self.step_Goal == 2): # - Đáp ứng góc cuối.
			self.complete_anglePoint = 1
			if (abs(self.delta_anglePoint) <= self.accuracyAngle):
				self.complete_anglePoint = 1

		# -- Tính toán tốc độ.

	def test(self):
		poseRobot = Pose()
		poseGoal = Pose()

		poseRobot.position.x = 0
		poseRobot.position.y = 0

		poseRobot.orientation = self.euler_to_quaternion(pi)

		poseGoal.position.x = 0
		poseGoal.position.y = 0
		poseGoal.orientation = self.euler_to_quaternion(pi)

		self.convertCoordinate_robot_goal(poseRobot, poseGoal)		

	def run(self):
		self.goalNow.position.x = 1
		self.goalNow.position.y = -2
		self.goalNow.orientation = self.euler_to_quaternion(0)

		while not rospy.is_shutdown():
			self.poseFake_robot, self.poseFake_goal = self.convertCoordinate_robot_goal(self.poseRobot, self.goalNow)

			if (self.isReceived_poseRobot == 1):
				self.folowGoal_keepCorner(self.poseFake_robot, self.poseFake_goal, 1, 0)

			self.cmd_vel.linear.x = self.velocities_lv1.linear_x
			self.cmd_vel.linear.y = self.velocities_lv1.linear_y
			self.cmd_vel.angular.z = self.velocities_lv1.angular_z
			self.pub_cmdVel.publish(self.cmd_vel)
			self.rate.sleep()
		# print('Programer stopped')


def main():
	print('Starting main program')

	program = testRun()
	program.run()

	print('Exiting main program')	

if __name__ == '__main__':
    main()	
