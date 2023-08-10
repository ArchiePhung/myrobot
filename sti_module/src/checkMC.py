#!/usr/bin/env python
# author : Anh Tuan - 2-10-2020

import rospy
from std_msgs.msg import Bool

class Start_launch:
    def __init__(self):
        rospy.init_node('checkMC', anonymous=True)
        self.rate = rospy.Rate(10)
   
        self.checkMC    = rospy.Publisher('/check_mc', Bool, queue_size=100)

    def raa(self): 
        while not rospy.is_shutdown():

			self.checkMC.publish(True)

			self.rate.sleep()

def main():
    
    try:
        m = Start_launch()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()