#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
App su dung cho viec test mach 
Developer: Phung Quy Duong
Company: STI
Start date: 12/4/2023
Latest Modify: 27/5/2023
    
Chức năng của node :
   + Check việc kết nối của các mạch với máy tính. 
"""

import roslib
roslib.load_manifest('launch_pkg')
import rospy

from message_pkg.msg import *
from geometry_msgs.msg import Twist
import time
import rospy
from std_msgs.msg import Int8
from ros_canbus.msg import *
import os
import subprocess, platform

class CheckPhysical:
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('check_physical', anonymous=True)
        self.rate = rospy.Rate(1)
        
        self.port_rtc = rospy.get_param('port_rtc', '')   
        self.port_board  = rospy.get_param('port_board', '')

        self.pub_statusPort = rospy.Publisher('/status_port', Status_port, queue_size= 50)
        self.statusPort = Status_port()

    def usbSerial_check(self, nameport):
        try:
            output = subprocess.check_output("ls {} {} {} {}".format('/dev/','|','grep', nameport ), shell=True)
            locate = str(output).find(nameport)
            if locate != -1:
                return 1
            return 0
        except Exception as e:
            return 0
        
    def run(self):
        while not rospy.is_shutdown():
            # -- rtc board -- 
            if self.usbSerial_check(self.port_rtc) == 1:
                self.statusPort.rtcPort_status = True
            else:
                self.statusPort.rtcPort_status = False
            
            # -- board test -- 
            if self.usbSerial_check(self.port_board) == 1:
                self.statusPort.boardPort_status = True
            else:
                self.statusPort.boardPort_status = False

            self.pub_statusPort.publish(self.statusPort)
            self.rate.sleep()

def main():
    print('Program starting')
    try:
        program = CheckPhysical()
        program.run()
    except rospy.ROSInterruptException:
        pass
    print('Programer stopped')

if __name__ == '__main__':
    main()