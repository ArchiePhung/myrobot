#!/usr/bin/env python
# update : BEE - 04-01-2023

import serial
import os
from message_pkg.msg import Status_port
import roslaunch
import rospy
import string
import subprocess, platform
import time

class CheckPhysical:
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('check_physical', anonymous=True)
        self.rate = rospy.Rate(1)

        self.port_lms100 = rospy.get_param('port_lms100', '')   
        self.port_tim551 = rospy.get_param('port_tim551', '')   
        self.port_camera = rospy.get_param('port_camera', '')  
        self.port_board  = rospy.get_param('port_board', '')    
        self.port_motor  = rospy.get_param('port_motor', '')
        self.port_imu    = rospy.get_param('port_imu', '')

        self.pub_statusPort = rospy.Publisher('/status_port', Status_port, queue_size= 50)
        self.statusPort = Status_port()

        self.pre_time = time.time()

    def ethernet_check(self, address):
        try:
            # print address
            output = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
            # print(output)
            result = output.find('time=') 
            # print ("time",result ) # yes : >0 
            if result != -1:
                return 1
            return 0
        except Exception, e:
            return 0

    def usbSerial_check_c3(self, nameport):
        try:
            output = subprocess.check_output("ls -{} {} {} {} {}".format('l','/dev/','|','grep', nameport ), shell=True)
            vitri = output.find("stibase")

            name = output[(vitri):(vitri+len(nameport))]
            # print(name)
            if name == nameport:
                return 1
            return 0
        except Exception, e:
            return 0

    def usbSerial_check(self, nameport):
        try:
            output = subprocess.check_output("ls {} {} {} {}".format('/dev/','|','grep', nameport ), shell=True)
            locate = output.find(nameport)
            if locate != -1:
                return 1
            return 0
        except Exception, e:
            return 0

    def usbCamera_check(self,nameport):
        try:
            output = subprocess.check_output("rs-enumerate-devices -{}".format('s'), shell=True)
            # print(output)
            vitri = output.find(nameport[1:])
            # name = output[(vitri):(vitri+len(nameport))]
            # print(result)
            # print(nameport[1:])
            if vitri > 1 : return 1
        except Exception, e:
            # print("no port")
            return 0

    def run(self):

        while not rospy.is_shutdown():
          # -- LMS100
            self.pre_time = time.time()
            if self.ethernet_check(self.port_lms100) == 1:
                self.statusPort.lms100 = True
            else:
                self.statusPort.lms100 = False
            t = time.time() - self.pre_time

          # -- TiM551
            if self.ethernet_check(self.port_tim551) == 1:
                self.statusPort.tim551 = True
            else:
                self.statusPort.tim551 = False

        #   # -- D435
            if self.usbCamera_check(self.port_camera) == 1:
                self.statusPort.camera = True
            else:
                self.statusPort.camera = False
            
          # -- BOARD        
            if self.usbSerial_check(self.port_board) == 1:
                self.statusPort.board = True
            else:
                self.statusPort.board = False

          # -- Motor
            if self.usbSerial_check(self.port_motor) == 1:
                self.statusPort.motor = True
            else:
                self.statusPort.motor = False

          # -- IMU     
            if self.usbSerial_check(self.port_imu) == 1:
                 self.statusPort.imu = True
            else:
                self.statusPort.imu = False
            
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