#!/usr/bin/env python
# author : Anh Tuan - 4-1-2021
'''
launch node apriltag_ros when move Parking or Setpose 
launch node via cmd with python 
D435.launch : 8% CPU 
apriltag_ros : tag_threads : 8 -> chiem 17% cpu 
apriltag_ros : tag_threads : 2 -> chiem 10% cpu 
apriltag_ros : tag_threads : 1 -> chiem 7%  cpu 
'''

import os
from sti_msgs.msg import  ManageLaunch
import rospy

class Start_launch:
	def __init__(self):
		rospy.init_node('reconnect', anonymous=True)
		self.rate = rospy.Rate(10)

		self.lms100_port = rospy.get_param('~lms100_port','')      
		self.mana_info    = rospy.Publisher('/manage_status', ManageLaunch, queue_size=100)
		self.stt = ManageLaunch()

		# variable check 
		# self.lms100_check = False


   
	def call_check0(self,data): 
		# self.stt = data
		pass

	

	def raa(self): 
		step = 1
		while not rospy.is_shutdown():
			if step == 1 : 
				step = 2 
			if step == 2 : 
				print("hihi")

			self.rate.sleep()

def main():
    
    try:
        m = Start_launch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()