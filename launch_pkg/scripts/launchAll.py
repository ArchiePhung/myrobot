#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
App su dung cho viec test mach 
Developer: Phung Quy Duong
Company: STI
Start date: 12/4/2023
Latest Modify: 27/5/2023
    
Chức năng của node:
   + Khởi động từng node 
   + Báo lỗi và dừng nếu node bị lỗi. 
"""

from std_msgs.msg import Int16, Int8

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

class launcher():
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('launcher', anonymous=False)
        self.rate = rospy.Rate(10)

        self.count_node = 0
        self.notification = ''
        self.step = 0
        self.timeWait = 1 # s 0.4

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

        # -- module - reconnectBase.
        self.path_reconnectBase = rospy.get_param('path_reconnectBase', '')
        self.launch_reconnectBase = Launch(self.path_reconnectBase)
        rospy.Subscriber('/status_reconnectBase', Status_reconnect, self.callBack_reconnectBase)
        self.is_reconnectBase = 1
        self.count_node += 1

        # -- module - RTC_ORIGIN.
        self.path_rtcf1 = rospy.get_param('path_rtcf1', '')
        self.launch_rtcf1 = Launch(self.path_rtcf1)
        rospy.Subscriber('/RTCF1_status', RTCF1_status, self.callBack_rtcf1)
        self.is_rtcf1 = 0
        self.count_node += 1

        # -- module - convert.
        self.path_convertCAN = rospy.get_param('path_convert', '')
        self.launch_convertCAN = Launch(self.path_convertCAN)
        rospy.Subscriber('/CAN_send', CAN_send, self.callBack_convertCAN)
        self.is_convertCAN = 0
        self.count_node += 1

        # -- module - appbackend.
        self.path_appbackend = rospy.get_param('path_appbackend', '')
        self.launch_appbackend = Launch(self.path_appbackend)
        rospy.Subscriber('/App_Lbv_btf', App_lbv_backTofront, self.callBack_appbackend)
        self.is_appbackend = 0
        self.count_node += 1

        # -- module - appfrontend.
        self.path_appfrontend = rospy.get_param('path_appfrontend', '')
        self.launch_appfrontend = Launch(self.path_appfrontend)
        rospy.Subscriber('/App_button', App_button_test, self.callBack_appfrontend)
        self.is_appfrontend = 0
        self.count_node += 1

        # -- ko dc xoa
        self.count_node += 1
        
    def callBack_firstWork(self, data):
        self.is_firstWork = 1

    def callBack_checkPort(self, data):
        self.is_checkPort = 1

    def callBack_reconnectBase(self, data):
        self.is_reconnectBase = 1

    def callBack_rtcf1(self, data):
        self.is_rtcf1 = 1

    def callBack_convertCAN(self, data):
        self.is_convertCAN = 1
    
    def callBack_appbackend(self, data):
        self.is_appbackend = 1
    
    def callBack_appfrontend(self, data):
        self.is_appfrontend = 1

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
            if (self.step == 1):
                self.notification = 'launch_checkPort'
                self.launch_checkPort.start()
                if (self.is_checkPort == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            elif (self.step == 2):
                self.notification = 'launch_app_frontend'
                self.launch_appfrontend.start()
                if (self.is_appfrontend == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- reconnectBase
            elif (self.step == 3):
                self.notification = 'launch_reconnectBase'
                self.launch_reconnectBase.start()
                if (self.is_reconnectBase == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            elif (self.step == 4):
                self.notification = 'launch_rtc_origin'
                self.launch_rtcf1.start()
                if (self.is_rtcf1 == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            elif (self.step == 5):
                self.notification = 'launch_convert_CAN'
                self.launch_convertCAN.start()
                if (self.is_convertCAN == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            elif (self.step == 6):
                self.notification = 'launch_app_backend'
                self.launch_appbackend.start()
                if (self.is_appbackend == 1):
                    self.step += 1
                    time.sleep(self.timeWait)

            # -- Completed
            elif (self.step == 7):
                self.notification = 'Completed!'

            # -- -- PUBLISH STATUS
            # self.stausLaunch.persent = int((self.step/self.count_node)*100.)
            self.stausLaunch.persent = int((self.step/7)*100.)
            self.stausLaunch.position = self.step
            self.stausLaunch.notification = self.notification
            self.pub_stausLaunch.publish(self.stausLaunch)
            # time.sleep(0.1)
            self.rate.sleep()

def main():
    print('Program starting')
    try:
        program = launcher()
        program.run()
    except rospy.ROSInterruptException:
        pass
    print('Programer stopped')

if __name__ == '__main__':
    main()
