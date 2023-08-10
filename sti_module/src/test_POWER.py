#!/usr/bin/env python
# license removed for brevity

import rospy
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sti_msgs.msg import *
from geometry_msgs.msg import PoseStamped
import time
import struct

#goal2_vao
goal2_vao = PoseStamped()
mb_read = Modbus_4x_read()
toyo_in = 0b00000000 
toyo_out = 0b00000000 
class test:
	def __init__(self):
		rospy.init_node('test_goal_control',anonymous=True)
		#giao tiep
		self.pub1 = rospy.Publisher('/parking_control', Parking_control, queue_size=10) 
		self.pub2 = rospy.Publisher('/goal_control', Goal_control, queue_size=10)
		self.pub3 = rospy.Publisher('/setpose_control', Setpose_control, queue_size=100)
		self.pub4 = rospy.Publisher('/modbus_4x_write', Modbus_4x_write, queue_size=100)
		rospy.Subscriber("/modbus_4x_read", Modbus_4x_read, self.callSup)

		self.pub5 = rospy.Publisher('/mission_control',Mission_control , queue_size=50)

		self.bt1_out = 0b0000000000000001
		self.bt1_in  = 0b0000000000000010
		self.bt2_out = 0b0000000000000100
		self.bt2_in  = 0b0000000000001000
		self.bt_stop = 0b0000000000000000

		self.SS_BT1_1 = bool()
		self.SS_BT1_1 = bool()
		self.SS_BT1_2 = bool()
		self.SS_BT2_1 = bool()
		self.SS_BT2_2 = bool()
		self.BT_EN    = bool()
		self.SS_ngoai = bool()

	def mission(self):
		new_mission = Mission_control()
		new_mission.enable = 0
		new_mission.mission  = 0
		new_mission.lifttable  = 0
		new_mission.lighting = 2
		new_mission.speaker = 2
		for i in range(3):
			self.pub5.publish(new_mission)
			print "hii"
			rospy.sleep(0.1)

	def chuyendoi(self, data, vitri):
		a = 0b0000000000000001
		a = a << vitri 
		if ( data & a ) > 1 : return True
		else : return False 

	def callSup(self,data):
		global mb_read
		mb_read = data
		self.SS_BT1_1 = self.chuyendoi(mb_read.Sie_in,1) 
		self.SS_BT1_2 = self.chuyendoi(mb_read.Sie_in,2) 
		self.SS_BT2_1 = self.chuyendoi(mb_read.Sie_in,4) 
		self.SS_BT2_2 = self.chuyendoi(mb_read.Sie_in,3) 
		self.BT_EN    = self.chuyendoi(mb_read.Sie_in,7) 
		self.SS_ngoai = self.chuyendoi(mb_read.BT_SIE,8)

	def bangtai(self):
		mb = Modbus_4x_write()
		global mb_read , toyo_in, toyo_out
		print self.SS_BT1_1 ,self.SS_BT1_2 ,self.SS_BT2_1,self.SS_BT2_2,self.BT_EN ,self.SS_ngoai
		dem  = 1 
		if dem == 1 : 
			if self.BT_EN == False : # bt dang hoat dong
				mb.BT_SIE_OUT = int(self.bt1_in)
				mb.BN_FC = 2 # nang
				mb.BT_DC_1 = 2 # q.trai
				self.pub4.publish(mb)
				while self.SS_BT1_2 == True : 
					print " wait1 " , self.SS_BT1_1
					pass 
				else : 
					mb.BT_SIE_OUT = int(self.bt_stop)
					mb.BT_DC_1 = 0 # stop
					self.pub4.publish(mb)
					# rospy.sleep(1)
					dem = 2
		if dem == 2 :
			mb.BT_SIE_OUT = int(self.bt1_out)
			mb.BT_DC_1 = 1 # q.phai
			self.pub4.publish(mb)
			while self.SS_ngoai == True : 
				print " wait2 " , self.SS_ngoai
				pass 
			else : 
				mb.BT_SIE_OUT = int(self.bt_stop)
				mb.BT_DC_1 = 0 # stop
				self.pub4.publish(mb)
				# rospy.sleep(1)
				dem = 3
		
		if dem == 3 : 
			print "hihi"
			return True 


		# print bin(mb_read.BT_SIE)

	def modbus(self):
		mb = Modbus_4x_write()
		mb.Led_FC = 3
		mb.BN_FC = 1
		mb.COI_FC = 0
		# mb.BT_SIE_OUT = int(self.bt_stop)

		# mb.Cmd_move = 17
		for i in range(5):
			self.pub4.publish(mb)
			print "hiijhsjhdsj"
			rospy.sleep(0.1)

	def setpose(self):
		sp = Setpose_control()
		sp.setposeTag = 11
		sp.distance = 5
		for i in range(5):
			self.pub3.publish(sp)
			print "hii"
			rospy.sleep(0.1)

	def enable_park(self):
		park = Parking_control()
		park.idTag = 18
		park.tolerance = 0.04
		park.distance = 0.3
		for i in range(3):
			self.pub1.publish(park)
			print "hii"
			rospy.sleep(0.1)

	def pub_goal_control(self):
		g_control = Goal_control()
		g_control.send_goal = goal2_vao
		g_control.enable_move = 2
		g_control.vel_x = 0.1
		g_control.vel_theta = 0.5
		g_control.tolerance_xy = 0.1
		g_control.tolerance_theta = 0.1
		g_control.planner_frequency = 5
		g_control.backward = 0.4
		for i in range(3):
			self.pub2.publish(g_control)
			print "hii"
			rospy.sleep(0.1)

    def test_power(self):
        p = POWER_request()

if __name__ == '__main__':
# global mb_read 
	a = test()

	goal2_vao.header.frame_id    = "map"  
	goal2_vao.header.stamp = rospy.Time.now()
	goal2_vao.pose.position.x    = -0.369
	goal2_vao.pose.position.y    = -0.037
	goal2_vao.pose.orientation.z = 0.75
	goal2_vao.pose.orientation.w = 0.662
	# a.pub_goal_control()
	# a.enable_park()
	# a.setpose()
	# a.modbus()
	# a.bangtai()
	# rospy.sleep (2)
	# a.bangtai()
	# a.mission()
    a.test_power()



	# rospy.spinOnce()
