#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
App điều khiển AGV robot SLAM 
Developer: Phung Quy Duong
Company: STI
Start date: 28/06/2023
Latest Modify: 

Chức năng (front_end ):
   + Hiển thị dữ liệu:
   + Xác thực các sự kiện nhấn giữ nút nhấn

"""

import sys
from math import sin , cos , pi , atan2
import time
import threading
import signal

from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication, QWidget, QMainWindow, QLineEdit, QTableWidgetItem, QFileDialog, QMessageBox
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer, QDateTime, Qt

import sqlite3

sys.path.append('/home/archiep/robot_ws/devel/lib/python3/dist-packages')
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

import roslib
import rospy
from message_pkg.msg import *
# from geometry_msgs import Twist
from datetime import datetime

class WelcomeScreen(QDialog):
	def __init__(self):
		super(WelcomeScreen, self).__init__()
		loadUi("/home/archiep/robot_ws/src/app_ros/interface/app.ui", self)
		
		# -- add message -- 
		self.data_app_request = App_request()
		
		self.launch_init()
		self.param_init()
		self.app_init()
		self.modeHand_init()

		# -- Timer updata data
		timer = QTimer(self)
		timer.timeout.connect(self.processOneThing)
		timer.start(100)

	def processOneThing(self):
		self.controlShow_followMode()

	def param_init(self):
		self.vel_max = 0.8 
		self.data_app_request.mode = self.modeRun_run        # robot run
		self.data_app_request.action = 0

		self.vel_now = 0.32

	def app_init(self):
		vel_percent = int((self.vel_now /self.vel_max)*100.0)
		self.pgb_vel.setValue(vel_percent)
	
# ------------------------------------------------------------ LAUNCHING GUI--------------------------------------------------- # 		
	def launch_init(self):
		# -- control app -- 
		self.modeRuning = 0
		self.modeRun_launch = 0
		self.modeRun_run = 1
		self.modeRun_settings = 2
		self.modeRuning = self.modeRun_run

		self.lbv_launching = ""
		self.lbv_numberLaunch = ""
		self.percentLaunch = 0

	def controlShow_followMode(self):
		# --
		if (self.modeRuning == self.modeRun_launch): # -- Khoi Dong
			self.fr_launch.show()
			self.fr_run.hide()
			self.show_launch()

		elif self.modeRuning == self.modeRun_run:
			self.fr_run.show()
			self.fr_launch.hide()
			self.fr_control.show()
			self.fr_setting.hide()

		elif self.modeRuning == self.modeRun_settings:
			self.fr_run.show()
			self.fr_launch.hide()
			self.fr_control.hide()
			self.fr_setting.show()

	def show_launch(self):
		self.lbv_node_start.setText(self.lbv_launching)
		self.lbv_node_number.setText(str(self.lbv_numberLaunch))
		# --
		value = self.percentLaunch
		if (value < 0):
			value = 0

		if (value > 100):
			value = 100

		self.pb_launch.setValue(value)
		
		if value == 100:
			self.modeRuning = self.modeRun_run
			time.sleep(2)

# --------------------------------------------------------------RUN GUI -------------------------------------------------------#
	def modeHand_init(self):
		self.bt_rotation_left.pressed.connect(self.pressed_bt_rotation_left)
		self.bt_backwards.pressed.connect(self.pressed_bt_backwards)
		self.bt_rotation_right.pressed.connect(self.pressed_bt_rotation_right)
		self.bt_forwards.pressed.connect(self.pressed_bt_forwards)
		self.bt_stop.pressed.connect(self.pressed_bt_stop)

		self.bt_vel_increase.clicked.connect(self.clicked_bt_vel_increase)
		self.bt_vel_decrease.clicked.connect(self.clicked_bt_vel_decrease)

	def pressed_bt_forwards(self):
		self.data_app_request.action = 1
		self.data_app_request.vel = self.vel_now                  
	
	def pressed_bt_rotation_right(self):
		self.data_app_request.action = 2               
		self.data_app_request.vel = self.vel_now
	
	def pressed_bt_backwards(self):
		self.data_app_request.action = 3
		self.data_app_request.vel = self.vel_now                  

	def pressed_bt_rotation_left(self):
		self.data_app_request.action = 4              
		self.data_app_request.vel = self.vel_now
	
	def pressed_bt_stop(self):
		self.data_app_request.action = 0
		self.data_app_request.vel = 0                 

	def clicked_bt_vel_increase(self):
		self.vel_now += 0.16
		if self.vel_now > 0.8:
			self.vel_now = 0.8 
		
		vel_percent = int((self.vel_now/self.vel_max)*100.0)
		self.pgb_vel.setValue(vel_percent)

		self.data_app_request.vel = self.vel_now

	def clicked_bt_vel_decrease(self):
		self.vel_now -= 0.16
		if self.vel_now < 0.08:
			self.vel_now = 0.08
		
		vel_percent = int((self.vel_now/self.vel_max)*100.0)
		self.pgb_vel.setValue(vel_percent)

		self.data_app_request.vel = self.vel_now

	def out(self):
		QApplication.quit()
		print('out')
	# ---------------------------
	
class Program(threading.Thread):
	def __init__(self, threadID):
		# -- for thread -- 
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.shutdown_flag = threading.Event()
        
		# -- for giao diện app -- 
		self.app = QApplication(sys.argv)
		self.welcomeScreen = WelcomeScreen()
		screen = self.app.primaryScreen()

		size = screen.size()
		print('Size: %d x %d' % (size.width(), size.height()))

		self.widget = QtWidgets.QStackedWidget()
		self.widget.addWidget(self.welcomeScreen)
		self.widget.setFixedHeight(558)
		self.widget.setFixedWidth(1024)

		# -- node init -- 
		rospy.init_node('app_frontend', anonymous=False)
		self.rate_hz = 2
		self.rate = rospy.Rate(self.rate_hz)

		# -- node publisher -- 
		self.pub_appRequest = rospy.Publisher('/App_request', App_request, queue_size= 10)
		self.data_appRequest = App_request()
	
		# -- node subcriber -- 
		# -- biến toàn cục -- 
		self.is_exist = 1           

	def run_screen(self):
		self.widget.show()
		try:
			# print ("run 1")
			sys.exit(self.app.exec_())
			# print ("run 2")
		except:
			pass
			# print("Exiting 1")
		self.is_exist = 0

	def kill_app(self):
		self.welcomeScreen.out()
		self.is_exist = 0

	def exec_launch(self):
		self.welcomeScreen.percentLaunch = self.data_launchStatus.persent
		self.welcomeScreen.lbv_launching = self.data_launchStatus.notification
		self.welcomeScreen.lbv_numberLaunch = self.data_launchStatus.position

	def exec_modeHand(self):
		self.data_appRequest.mode = self.welcomeScreen.data_app_request.mode
		self.data_appRequest.action = self.welcomeScreen.data_app_request.action
		self.data_appRequest.vel = self.welcomeScreen.data_app_request.vel

	def run(self):
		while (not self.shutdown_flag.is_set()) and (not rospy.is_shutdown()) and (self.is_exist == 1):
			self.exec_modeHand()
			
			self.pub_appRequest.publish(self.data_appRequest)
			self.rate.sleep()

		self.is_exist = 0
		self.kill_app()

		print('Thread #%s stopped' % self.threadID)

class ServiceExit(Exception):
	"""
	Custom exception which is used to trigger the clean exit
	of all running threads and the main program.
	"""
	pass
 
def service_shutdown(signum, frame):
	print('Caught signal %d' % signum)
	raise ServiceExit

def main():
	# Register the signal handlers
	signal.signal(signal.SIGTERM, service_shutdown)
	signal.signal(signal.SIGINT, service_shutdown)

	print('Starting main program')

	# Start the job threads
	try:
		thread1 = Program(1)
		thread1.start()

		# Keep the main thread running, otherwise signals are ignored.
		thread1.run_screen()
		thread1.is_exist = 0

	except ServiceExit:
		thread1.shutdown_flag.set()
		thread1.join()
		print('Exiting main program')
 
if __name__ == '__main__':
	main()