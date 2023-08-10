#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
App su dung cho viec test mach 
Developer: Phung Quy Duong
Company: STI
Start date: 12/4/2023
Latest Modify: 27/5/2023
    
Chức năng của node (back_end):
   + Nhận tín hiệu sự kiện từ app, xử lý và cung cấp cho node khác 
   + Nhận tín hiệu từ các node khác và gửi cho node app 
"""

import roslib
roslib.load_manifest('app_ros')
import rospy

from message_pkg.msg import *
from geometry_msgs.msg import Twist
import time
import rospy
from std_msgs.msg import Int8
from ros_canbus.msg import *
import os

class APP_TEST_BACKEND():
	def __init__(self):
		rospy.init_node('app_backend', anonymous=False)
		self.rate = rospy.Rate(50)

		# ------------- PARAMETER ------------- #
		# self.path_rtc_origin = rospy.get_param('path_rtc_origin')
		# self.path_rtc  = rospy.get_param('path_rtc')
		# self.path_main = rospy.get_param('path_main')
		# self.path_oc   = rospy.get_param('path_oc')
		# self.path_hc   = rospy.get_param('path_hc')
		# self.path_cpd  = rospy.get_param('path_cpd')

		self.cm_enable = rospy.get_param('cm_enable')
		self.cm_enable1 = rospy.get_param('cm_enable1')
		self.cm_upload = rospy.get_param('cm_upload')

		# ------------- ROS ------------- #
		# - PUBLISH
		self.pub_lbcApp = rospy.Publisher("/App_LabelColor", App_color_test, queue_size = 10)
		self.data_lbcApp = App_color_test()
		
		self.pub_lbvApp_btf = rospy.Publisher("/App_Lbv_btf", App_lbv_backTofront, queue_size = 10)
		self.data_lbvApp_btf = App_lbv_backTofront()

		self.pub_RequestRTC = rospy.Publisher("/request_rtc", Control_rtc, queue_size = 10)
		self.data_requestRTC = Control_rtc()

		self.pub_RequestMAIN = rospy.Publisher("/request_main", Control_main, queue_size = 10)
		self.data_requestMAIN = Control_main()

		self.pub_RequestOC = rospy.Publisher("/request_oc", Control_oc, queue_size = 10)
		self.data_requestOC = Control_oc()

		self.pub_RequestHC = rospy.Publisher("/request_hc", Control_hc, queue_size = 10)
		self.data_requestHC = Control_hc()

		self.pub_RequestCPD = rospy.Publisher("/request_cpd", Control_cpd, queue_size = 10)
		self.data_requestCPD = Control_cpd()

		# - SUBCRIBER
		rospy.Subscriber('/App_button', App_button_test, self.callBack_btApp)
		self.data_buttonApp = App_button_test()

		rospy.Subscriber('/App_time', App_lbv_frontToback, self.callBack_lbvApp)
		self.data_lbvApp_ftb = App_lbv_frontToback()
		
		rospy.Subscriber("/RTC_info", Status_rtc, self.callBack_RTCinfo)
		self.data_RTCinfo = Status_rtc()

		rospy.Subscriber("/MAIN_info", Status_main, self.callBack_MAINinfo)
		self.data_MAINinfo = Status_main()

		rospy.Subscriber("/OC_info", Status_oc, self.callBack_OCinfo)
		self.data_OCinfo = Status_oc()

		rospy.Subscriber("/HC_info", Status_hc, self.callBack_HCinfo)
		self.data_HCinfo = Status_hc()

		rospy.Subscriber("/CPD_info", Status_cpd, self.callBack_CPDinfo)
		self.data_CPDinfo = Status_cpd()

		rospy.Subscriber("/RTCF1_status", RTCF1_status, self.callBack_RTCF1status)
		self.data_RTCF1status = RTCF1_status()
		
		# ------------- VAR ------------- #
		# -- upload code --
		self.first_rtc_upload = 1
		self.first_main_upload = 1
		self.first_oc_upload = 1
		self.first_hc_upload = 1
		self.first_cpd_upload = 1

		# -- rtc var -- 
		self.time_rtc_count = time.time()
		self.rtc_step = 0
		self.rtc_timeWaitDisplay = time.time()

		# -- main var -- 
		self.time_main_count = time.time()
		self.main_step = 0
		self.main_timeWaitDisplay = time.time()

		# -- oc var -- 
		self.time_oc_count = time.time()
		self.oc_step = 0
		self.oc_timeWaitDisplay = time.time()

		# -- hc var -- 
		self.time_hc_count = time.time()
		self.hc_step = 0
		self.hc_timeWaitDisplay = time.time()

		# -- cpd var -- 
		self.time_cpd_count = time.time()
		self.cpd_step = 0
		self.cpd_timeWaitDisplay = time.time()

		# -- settings --
		self.time_status = time.time()
		self.time_recv_status = time.time()
		self.pre_status_value = 0

	def callBack_btApp(self, data):
		self.data_buttonApp = data

	def callBack_lbvApp(self, data):
		self.data_lbvApp_ftb = data
	
	def callBack_RTCinfo(self, data):
		self.data_RTCinfo = data

	def callBack_MAINinfo(self, data):
		self.data_MAINinfo = data

	def callBack_OCinfo(self, data):
		self.data_OCinfo = data
	
	def callBack_HCinfo(self, data):
		self.data_HCinfo = data

	def callBack_CPDinfo(self, data):
		self.data_CPDinfo = data

	def callBack_RTCF1status(self, data):
		self.data_RTCF1status = data
		self.time_recv_status = time.time()

	def upload_func(self, path):
		os.chdir(path)
		os.system(self.cm_enable1)
		os.system(self.cm_upload)

	def exec_uploadtab(self):
		# -- rtc upload -- 
		if self.data_buttonApp.upload_rtc == True:
			if self.first_rtc_upload == 1:
				self.upload_func(self.data_lbvApp_ftb.rtc_path_upload)
				self.first_rtc_upload = 0
			
			self.data_lbcApp.lbc_upload_rtc_status = self.data_RTCinfo.rtc_upload_status
		else:
			self.data_lbcApp.lbc_upload_rtc_status = 0
			self.first_rtc_upload = 1

		# -- main upload -- 
		if self.data_buttonApp.upload_main == True:
			if self.first_main_upload == 1:
				self.upload_func(self.data_lbvApp_ftb.main_path_upload)
				self.first_main_upload = 0

			self.data_lbcApp.lbc_upload_main_status = self.data_MAINinfo.main_upload_status
		else:
			self.data_lbcApp.lbc_upload_main_status = 0
			self.first_main_upload = 1

		# -- OC upload -- 
		if self.data_buttonApp.upload_oc == True:
			if self.first_oc_upload == 1:
				self.upload_func(self.data_lbvApp_ftb.oc_path_upload)
				self.first_oc_upload = 0

			self.data_lbcApp.lbc_upload_oc_status = self.data_OCinfo.oc_upload_status
		else:
			self.data_lbcApp.lbc_upload_oc_status = 0
			self.first_oc_upload = 1

		# -- HC upload -- 
		if self.data_buttonApp.upload_hc == True:
			if self.first_hc_upload == 1:
				self.upload_func(self.data_lbvApp_ftb.hc_path_upload)
				self.first_hc_upload = 0

			self.data_lbcApp.lbc_upload_hc_status = self.data_HCinfo.hc_upload_status
		else:
			self.data_lbcApp.lbc_upload_hc_status = 0
			self.first_hc_upload = 1

		# -- CPD upload -- 
		if self.data_buttonApp.upload_cpd == True:
			if self.first_cpd_upload == 1:
				self.upload_func(self.data_lbvApp_ftb.cpd_path_upload)
				self.first_cpd_upload = 0

			self.data_lbcApp.lbc_upload_cpd_status = self.data_CPDinfo.cpd_upload_status

		else:
			self.data_lbcApp.lbc_upload_cpd_status = 0
			self.first_cpd_upload = 1

	def exec_rtctab(self):
		self.data_requestRTC.rtc_can_sec = self.data_buttonApp.rtc_can_sec
		self.data_requestRTC.rtc_can_sen = self.data_buttonApp.rtc_can_sen
		self.data_requestRTC.rtc_ros_sec = self.data_buttonApp.rtc_ros_sec
		self.data_requestRTC.rtc_ros_sen = self.data_buttonApp.rtc_ros_sen

		if self.data_buttonApp.rtc_can_write == True:                      
			if self.data_lbvApp_ftb.lbv_rtc_timeCan > 0:
				if self.rtc_step == 0:
					self.time_rtc_count = time.time()
					self.rtc_step = 1
					
				elif self.rtc_step == 1:
					if time.time() - self.time_rtc_count < 2:
						self.data_requestRTC.rtc_value_writeCan = 1	

						if self.data_RTCinfo.rtc_value_canRead == 1:
							self.rtc_step = 2
							self.time_rtc_count = time.time() 
					else:
						self.data_lbvApp_btf.rtc_statusCan = -1              
						self.rtc_step = 0

				elif self.rtc_step == 2:
					if time.time() - self.time_rtc_count < 2:
						self.data_requestRTC.rtc_value_writeCan = 2

						if self.data_RTCinfo.rtc_value_canRead == 2:
							self.rtc_step = 3
							self.time_rtc_count = time.time() 
					else: 
						self.data_lbvApp_btf.rtc_statusCan = -1               
						self.rtc_step = 0

				elif self.rtc_step == 3:
					if time.time() - self.time_rtc_count < 2:
						self.data_requestRTC.rtc_value_writeCan = 3

						if self.data_RTCinfo.rtc_value_canRead == 3:
							self.rtc_step = 4
							self.time_rtc_count = time.time()
					else:
						self.data_lbvApp_btf.rtc_statusCan = -1              
						self.rtc_step = 0

				elif self.rtc_step == 4:
					if time.time() - self.time_rtc_count < 2:
						self.data_requestRTC.rtc_value_writeCan = 4
							
						if self.data_RTCinfo.rtc_value_canRead == 4:
							self.rtc_step = 5
							self.time_rtc_count = time.time()
							
					else:
						self.data_lbvApp_btf.rtc_statusCan = -1               
						self.rtc_step = 0
				
				elif self.rtc_step == 5:
					if time.time() - self.time_rtc_count < 2:
						self.data_requestRTC.rtc_value_writeCan = 5
							
						if self.data_RTCinfo.rtc_value_canRead == 5:
							self.rtc_step = 0
							self.time_rtc_count = time.time()
							self.data_lbvApp_btf.rtc_statusCan = 1            
					else:
						self.data_lbvApp_btf.rtc_statusCan = -1          
						self.rtc_step = 0

			elif self.data_lbvApp_ftb.lbv_rtc_timeCan == 0:
				if time.time() - self.rtc_timeWaitDisplay >= 5:
					self.data_lbvApp_btf.rtc_statusCan = 0
					self.rtc_step = 0
					self.rtc_timeWaitDisplay = time.time()
	
	def exec_maintab(self):
		self.data_requestMAIN.main_charge = self.data_buttonApp.main_charge
		self.data_requestMAIN.main_emc_write = self.data_buttonApp.main_emc_write
		self.data_requestMAIN.main_emc_reset = self.data_buttonApp.main_emc_reset
		self.data_requestMAIN.main_off_power = self.data_buttonApp.main_off_power

		self.data_requestMAIN.main_sound1 = self.data_buttonApp.main_sound1
		self.data_requestMAIN.main_sound2 = self.data_buttonApp.main_sound2
		self.data_requestMAIN.main_sound3 = self.data_buttonApp.main_sound3
		self.data_requestMAIN.main_sound4 = self.data_buttonApp.main_sound4

		self.data_lbcApp.lbc_stsButton_reset = self.data_MAINinfo.stsButton_reset
		self.data_lbcApp.lbc_stsButton_power = self.data_MAINinfo.stsButton_power
		self.data_lbcApp.lbc_EMC_status = self.data_MAINinfo.EMC_status
		
		self.data_lbvApp_btf.main_voltages_analog = self.data_MAINinfo.voltages_analog
		self.data_lbvApp_btf.main_charge_analog = self.data_MAINinfo.charge_analog

		if self.data_buttonApp.main_can_write == True:                      
			if self.data_lbvApp_ftb.lbv_main_timeCan > 0:
				if self.main_step == 0:
					self.time_main_count = time.time()
					self.main_step = 1
					
				elif self.main_step == 1:
					if time.time() - self.time_main_count < 2:
						self.data_requestMAIN.main_value_writeCan = 1	

						if self.data_MAINinfo.main_value_canRead == 1:
							self.main_step = 2
							self.time_main_count = time.time() 
					else:
						self.data_lbvApp_btf.main_statusCan = -1              
						self.main_step = 0

				elif self.main_step == 2:
					if time.time() - self.time_main_count < 2:
						self.data_requestMAIN.main_value_writeCan = 2

						if self.data_MAINinfo.main_value_canRead == 2:
							self.main_step = 3
							self.time_main_count = time.time() 
					else: 
						self.data_lbvApp_btf.main_statusCan = -1               
						self.main_step = 0

				elif self.main_step == 3:
					if time.time() - self.time_main_count < 2:
						self.data_requestMAIN.main_value_writeCan = 3

						if self.data_MAINinfo.main_value_canRead == 3:
							self.main_step = 4
							self.time_main_count = time.time()
					else:
						self.data_lbvApp_btf.main_statusCan = -1              
						self.main_step = 0

				elif self.main_step == 4:
					if time.time() - self.time_main_count < 2:
						self.data_requestMAIN.main_value_writeCan = 4
							
						if self.data_MAINinfo.main_value_canRead == 4:
							self.main_step = 5
							self.time_main_count = time.time()
							
					else:
						self.data_lbvApp_btf.main_statusCan = -1               
						self.main_step = 0
				
				elif self.main_step == 5:
					if time.time() - self.time_main_count < 2:
						self.data_requestMAIN.main_value_writeCan = 5
							
						if self.data_MAINinfo.main_value_canRead == 5:
							self.main_step = 0
							self.time_main_count = time.time()
							self.data_lbvApp_btf.main_statusCan = 1            
					else:
						self.data_lbvApp_btf.main_statusCan = -1          
						self.main_step = 0

			elif self.data_lbvApp_ftb.lbv_main_timeCan == 0:
				if time.time() - self.main_timeWaitDisplay >= 5:
					self.data_lbvApp_btf.main_statusCan = 0
					self.main_step = 0
					self.main_timeWaitDisplay = time.time()
	
	def exec_octab(self):
		# -- OC DATA --
		self.data_lbcApp.lbc_oc_sensor1 = self.data_OCinfo.sensor_s1
		self.data_lbcApp.lbc_oc_sensor3 = self.data_OCinfo.sensor_s3
		self.data_lbcApp.lbc_oc_sensor4 = self.data_OCinfo.sensor_s4
		self.data_lbcApp.lbc_oc_sensor5 = self.data_OCinfo.sensor_s5
		self.data_lbcApp.lbc_oc_sensor6 = self.data_OCinfo.sensor_s6
		self.data_lbcApp.lbc_oc_sensor7 = self.data_OCinfo.sensor_s7

		# -- OC REQUEST -- 
		self.data_requestOC.manipulate_oc_bt_ccw = self.data_buttonApp.oc_bt_ccw
		self.data_requestOC.manipulate_oc_bt_cw = self.data_buttonApp.oc_bt_cw
		self.data_requestOC.manipulate_oc_bn_ccw = self.data_buttonApp.oc_bn_ccw
		self.data_requestOC.manipulate_oc_bn_cw = self.data_buttonApp.oc_bn_cw
		
		if self.data_buttonApp.oc_can_write == True:                      # truyền và nhận được CAN sau 5 lần thì có nghĩa CAN oke -> hiển thị lên App, ngược lại sẽ báo NG nếu ko nhận được ở bất cứ bước nào
			if self.data_lbvApp_ftb.lbv_oc_timeCan > 0:
				if self.oc_step == 0:
					self.time_oc_count = time.time()
					self.oc_step = 1
					
				elif self.oc_step == 1:
					if time.time() - self.time_oc_count < 2:
						self.data_requestOC.oc_value_writeCan = 1	

						if self.data_OCinfo.oc_value_canRead == 1:
							self.oc_step = 2
							self.time_oc_count = time.time() 
					else:
						self.data_lbvApp_btf.oc_statusCan = -1              # CAn is not good 
						self.oc_step = 0

				elif self.oc_step == 2:
					if time.time() - self.time_oc_count < 2:
						self.data_requestOC.oc_value_writeCan = 2

						if self.data_OCinfo.oc_value_canRead == 2:
							self.oc_step = 3
							self.time_oc_count = time.time() 
					else: 
						self.data_lbvApp_btf.oc_statusCan = -1               # CAn is not good 
						self.oc_step = 0

				elif self.oc_step == 3:
					if time.time() - self.time_oc_count < 2:
						self.data_requestOC.oc_value_writeCan = 3

						if self.data_OCinfo.oc_value_canRead == 3:
							self.oc_step = 4
							self.time_oc_count = time.time()
					else:
						self.data_lbvApp_btf.oc_statusCan = -1               # CAn is not good 
						self.oc_step = 0

				elif self.oc_step == 4:
					if time.time() - self.time_oc_count < 2:
						self.data_requestOC.oc_value_writeCan = 4
							
						if self.data_OCinfo.oc_value_canRead == 4:
							self.oc_step = 5
							self.time_oc_count = time.time()
							
					else:
						self.data_lbvApp_btf.oc_statusCan = -1               # CAn is not good 
						self.oc_step = 0
				
				elif self.oc_step == 5:
					if time.time() - self.time_oc_count < 2:
						self.data_requestOC.oc_value_writeCan = 5
							
						if self.data_OCinfo.oc_value_canRead == 5:
							self.oc_step = 0
							self.time_oc_count = time.time()
							self.data_lbvApp_btf.oc_statusCan = 1            # OC - CAN oke
					else:
						self.data_lbvApp_btf.oc_statusCan = -1           # CAn is not good 
						self.oc_step = 0

			elif self.data_lbvApp_ftb.lbv_oc_timeCan == 0:
				if time.time() - self.oc_timeWaitDisplay >= 5:
					self.data_lbvApp_btf.oc_statusCan = 0
					self.oc_step = 0
					self.oc_timeWaitDisplay = time.time()

	def exec_hctab(self):
		# -- hc status ---
		self.data_lbcApp.lbc_hc_sick_ahead0 = self.data_HCinfo.hc_sick_ahead0
		self.data_lbcApp.lbc_hc_sick_ahead1 = self.data_HCinfo.hc_sick_ahead1
		self.data_lbcApp.lbc_hc_sick_ahead2 = self.data_HCinfo.hc_sick_ahead2
		self.data_lbcApp.lbc_hc_sick_ahead3 = self.data_HCinfo.hc_sick_ahead3

		self.data_lbcApp.lbc_hc_sick_behind0 = self.data_HCinfo.hc_sick_behind0
		self.data_lbcApp.lbc_hc_sick_behind1 = self.data_HCinfo.hc_sick_behind1
		self.data_lbcApp.lbc_hc_sick_behind2 = self.data_HCinfo.hc_sick_behind2
		self.data_lbcApp.lbc_hc_sick_behind3 = self.data_HCinfo.hc_sick_behind3

		self.data_lbcApp.lbc_hc_badersock = self.data_HCinfo.hc_badersock

		# -- Hc request -- 
		self.data_requestHC.hc_led_red = self.data_buttonApp.hc_led_red
		self.data_requestHC.hc_led_green = self.data_buttonApp.hc_led_green
		self.data_requestHC.hc_led_blue = self.data_buttonApp.hc_led_blue
		self.data_requestHC.hc_sickzone1 = self.data_buttonApp.hc_sickzone1
		self.data_requestHC.hc_sickzone2 = self.data_buttonApp.hc_sickzone2

		# -- hc can -- 
		if self.data_buttonApp.hc_can_write == True:                      # truyền và nhận được CAN sau 5 lần thì có nghĩa CAN oke -> hiển thị lên App, ngược lại sẽ báo NG nếu ko nhận được ở bất cứ bước nào
			if self.data_lbvApp_ftb.lbv_hc_timeCan > 0:
				if self.hc_step == 0:
					self.time_hc_count = time.time()
					self.hc_step = 1
					
				elif self.hc_step == 1:
					if time.time() - self.time_hc_count < 2:
						self.data_requestHC.hc_value_writeCan = 1	

						if self.data_HCinfo.hc_value_canRead == 1:
							self.hc_step = 2
							self.time_hc_count = time.time() 
					else:
						self.data_lbvApp_btf.hc_statusCan = -1              # CAn is not good 
						self.hc_step = 0

				elif self.hc_step == 2:
					if time.time() - self.time_hc_count < 2:
						self.data_requestHC.hc_value_writeCan = 2

						if self.data_HCinfo.hc_value_canRead == 2:
							self.hc_step = 3
							self.time_hc_count = time.time() 
					else: 
						self.data_lbvApp_btf.hc_statusCan = -1               # CAn is not good 
						self.hc_step = 0

				elif self.hc_step == 3:
					if time.time() - self.time_hc_count < 2:
						self.data_requestHC.hc_value_writeCan = 3

						if self.data_HCinfo.hc_value_canRead == 3:
							self.hc_step = 4
							self.time_hc_count = time.time()
					else:
						self.data_lbvApp_btf.hc_statusCan = -1               # CAn is not good 
						self.hc_step = 0

				elif self.hc_step == 4:
					if time.time() - self.time_hc_count < 2:
						self.data_requestHC.hc_value_writeCan = 4
							
						if self.data_HCinfo.hc_value_canRead == 4:
							self.hc_step = 5
							self.time_hc_count = time.time()
							
					else:
						self.data_lbvApp_btf.hc_statusCan = -1               # CAn is not good 
						self.hc_step = 0
				
				elif self.hc_step == 5:
					if time.time() - self.time_hc_count < 2:
						self.data_requestHC.hc_value_writeCan = 5
							
						if self.data_HCinfo.hc_value_canRead == 5:
							self.hc_step = 0
							self.time_hc_count = time.time()
							self.data_lbvApp_btf.hc_statusCan = 1            # hc - CAN oke
					else:
						self.data_lbvApp_btf.hc_statusCan = -1           # CAn is not good 
						self.hc_step = 0

			elif self.data_lbvApp_ftb.lbv_hc_timeCan == 0:
				if time.time() - self.hc_timeWaitDisplay >= 5:
					self.data_lbvApp_btf.hc_statusCan = 0
					self.hc_step = 0
					self.hc_timeWaitDisplay = time.time()

	def exec_cpdtab(self):
		# -- cpd status ---
		self.data_lbcApp.lbc_cpd_input1 = self.data_CPDinfo.cpd_input1
		self.data_lbcApp.lbc_cpd_input2 = self.data_CPDinfo.cpd_input2
		self.data_lbcApp.lbc_cpd_input3 = self.data_CPDinfo.cpd_input3
		self.data_lbcApp.lbc_cpd_input4 = self.data_CPDinfo.cpd_input4
		self.data_lbcApp.lbc_cpd_input5 = self.data_CPDinfo.cpd_input5
		self.data_lbcApp.lbc_cpd_input6 = self.data_CPDinfo.cpd_input6
		self.data_lbcApp.lbc_cpd_input7 = self.data_CPDinfo.cpd_input7
		self.data_lbcApp.lbc_cpd_input8 = self.data_CPDinfo.cpd_input8
		self.data_lbcApp.lbc_cpd_input9 = self.data_CPDinfo.cpd_input9
		self.data_lbcApp.lbc_cpd_input10 = self.data_CPDinfo.cpd_input10
		self.data_lbcApp.lbc_cpd_input11 = self.data_CPDinfo.cpd_input11
		self.data_lbcApp.lbc_cpd_input12 = self.data_CPDinfo.cpd_input12

		# -- cpd request -- 
		self.data_requestCPD.cpd_output1 = self.data_buttonApp.cpd_output1
		self.data_requestCPD.cpd_output2 = self.data_buttonApp.cpd_output2
		self.data_requestCPD.cpd_output3 = self.data_buttonApp.cpd_output3
		self.data_requestCPD.cpd_output4 = self.data_buttonApp.cpd_output4
		self.data_requestCPD.cpd_output5 = self.data_buttonApp.cpd_output5
		self.data_requestCPD.cpd_output6 = self.data_buttonApp.cpd_output6
		self.data_requestCPD.cpd_output7 = self.data_buttonApp.cpd_output7
		self.data_requestCPD.cpd_output8 = self.data_buttonApp.cpd_output8
		self.data_requestCPD.cpd_output9 = self.data_buttonApp.cpd_output9
		self.data_requestCPD.cpd_output10 = self.data_buttonApp.cpd_output10
		self.data_requestCPD.cpd_output11 = self.data_buttonApp.cpd_output11
		self.data_requestCPD.cpd_output12 = self.data_buttonApp.cpd_output12

		# -- cpd can -- 
		if self.data_buttonApp.cpd_can_write == True:                      # truyền và nhận được CAN sau 5 lần thì có nghĩa CAN oke -> hiển thị lên App, ngược lại sẽ báo NG nếu ko nhận được ở bất cứ bước nào
			if self.data_lbvApp_ftb.lbv_cpd_timeCan > 0:
				if self.cpd_step == 0:
					self.time_cpd_count = time.time()
					self.cpd_step = 1
					
				elif self.cpd_step == 1:
					if time.time() - self.time_cpd_count < 2:
						self.data_requestCPD.cpd_value_writeCan = 1	

						if self.data_CPDinfo.cpd_value_canRead == 1:
							self.cpd_step = 2
							self.time_cpd_count = time.time() 
					else:
						self.data_lbvApp_btf.cpd_statusCan = -1              # CAn is not good 
						self.cpd_step = 0

				elif self.cpd_step == 2:
					if time.time() - self.time_cpd_count < 2:
						self.data_requestCPD.cpd_value_writeCan = 2

						if self.data_CPDinfo.cpd_value_canRead == 2:
							self.cpd_step = 3
							self.time_cpd_count = time.time() 
					else: 
						self.data_lbvApp_btf.cpd_statusCan = -1               # CAn is not good 
						self.cpd_step = 0

				elif self.cpd_step == 3:
					if time.time() - self.time_cpd_count < 2:
						self.data_requestCPD.cpd_value_writeCan = 3

						if self.data_CPDinfo.cpd_value_canRead == 3:
							self.cpd_step = 4
							self.time_cpd_count = time.time()
					else:
						self.data_lbvApp_btf.cpd_statusCan = -1               # CAn is not good 
						self.cpd_step = 0

				elif self.cpd_step == 4:
					if time.time() - self.time_cpd_count < 2:
						self.data_requestCPD.cpd_value_writeCan = 4
							
						if self.data_CPDinfo.cpd_value_canRead == 4:
							self.cpd_step = 5
							self.time_cpd_count = time.time()
							
					else:
						self.data_lbvApp_btf.cpd_statusCan = -1               # CAn is not good 
						self.cpd_step = 0
				
				elif self.cpd_step == 5:
					if time.time() - self.time_cpd_count < 2:
						self.data_requestCPD.cpd_value_writeCan = 5
							
						if self.data_CPDinfo.cpd_value_canRead == 5:
							self.cpd_step = 0
							self.time_cpd_count = time.time()
							self.data_lbvApp_btf.cpd_statusCan = 1            # cpd - CAN oke
					else:
						self.data_lbvApp_btf.cpd_statusCan = -1           # CAn is not good 
						self.cpd_step = 0

			elif self.data_lbvApp_ftb.lbv_cpd_timeCan == 0:
				if time.time() - self.cpd_timeWaitDisplay >= 5:
					self.data_lbvApp_btf.cpd_statusCan = 0
					self.cpd_step = 0
					self.cpd_timeWaitDisplay = time.time()

	def exec_rtcf1(self):
		if time.time() - self.time_recv_status > 1:
			self.data_lbvApp_btf.rtcf1_run_status = -1           # send NG to app
		else:
			self.data_lbvApp_btf.rtcf1_run_status = 1            # send OK to app

	def exec_mainf1(self):
		pass

	def run(self):
		while not rospy.is_shutdown():
			# ------------------------------------------- UPLOAD-CODE TAB ------------------------------------------------#
			self.exec_uploadtab()
			# ---------------------------------------------- RTC BOARD ---------------------------------------------------#
			self.exec_rtctab()
			# ---------------------------------------------- MAIN BOARD --------------------------------------------------#
			self.exec_maintab()
			# ---------------------------------------------- OC BOARD ----------------------------------------------------#
			self.exec_octab()
			# ---------------------------------------------- HC BOARD ----------------------------------------------------#
			self.exec_hctab()
			# --------------------------------------------- CPD BOARD ----------------------------------------------------#
			self.exec_cpdtab()
			# ------------------------------------------ RTC ORIGIN BOARD ------------------------------------------------#
			self.exec_rtcf1()
			# ----------------------------------------- MAIN ORIGIN BOARD ------------------------------------------------#
			self.exec_mainf1()
			# --------------------------------------------- ROS PUBLISH  -------------------------------------------------#
			self.pub_lbvApp_btf.publish(self.data_lbvApp_btf)
			self.pub_lbcApp.publish(self.data_lbcApp)
			self.pub_RequestRTC.publish(self.data_requestRTC)
			self.pub_RequestMAIN.publish(self.data_requestMAIN)
			self.pub_RequestOC.publish(self.data_requestOC)
			self.pub_RequestHC.publish(self.data_requestHC)
			self.pub_RequestCPD.publish(self.data_requestCPD)
			self.rate.sleep()

def main():
	print('Program starting')
	program = APP_TEST_BACKEND()
	program.run()
	print('Programer stopped')

if __name__ == '__main__':
    main()