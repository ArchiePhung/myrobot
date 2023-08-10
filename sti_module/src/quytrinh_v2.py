#!/usr/bin/python

import threading
import time
import rospy
from std_msgs.msg import String

import sys
import struct
import string
import roslib
import serial
import signal
# clear costmap
from std_srvs.srv import Empty, EmptyRequest

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import *
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose, Twist
from sensor_msgs.msg import LaserScan

from math import pi as PI
from math import atan2, sin, cos, sqrt, fabs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import dynamic_reconfigure.client
import Queue
from sti_msgs.msg import *

#  pose Robot from Aruco
poseRbAr = PoseWithCovarianceStamped()  
enable_poseRbAr = False
#  pose Robot in Map
poseRbMa = Pose()
enable_poseRbMa = False

vel_robot = 0 
vel_0 = False
mb_read = Modbus_4x_read()
mb_control = Modbus_4x_write()
 
class quyTrinh(threading.Thread):

    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

        rospy.init_node('quytrinh', anonymous=True)

        #parking
        self.pub = rospy.Publisher('/parking_control', Parking_control, queue_size=10) 
        rospy.Subscriber("/parking_status", Parking_status, self.callSup)

        self.rate = rospy.Rate(1) # 1hz
        rospy.Subscriber("/posePoseRobotFromAruco", PoseWithCovarianceStamped, self.callSup1)
        rospy.Subscriber("/raw_vel", Velocities, self.callSup2)
        rospy.Subscriber("/robot_pose", Pose, self.callSup3)

        #cancel Goal
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        #clear_costmap
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        #set pose
        self.initpose_publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)
        #ReserMap hector
        self.ReserMap = rospy.Publisher('syscommand', String, queue_size = 100)

        # The shutdown_flag is a threading.Event object that
        # indicates whether the thread should be terminated.
        self.shutdown_flag = threading.Event()

        #modbus
        self.pub4 = rospy.Publisher('/modbus_4x_write', Modbus_4x_write, queue_size=100)
        rospy.Subscriber("/modbus_4x_read", Modbus_4x_read, self.callSup4)

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

        #bien
        self.park_info = 0
    
    def callSup(self,data):
        self.park_info = data.status

    def call_parking(self,idTag,tolerance,distance):
        park = Parking_control()
        park.idTag = idTag
        park.tolerance = tolerance
        park.distance = distance
        self.pub.publish(park)

    def callSup1(self, pose):
        global enable_poseRbAr
        global poseRbAr
        poseRbAr = pose
        enable_poseRbAr = True

    def locPoseRb(self, i):
        global enable_poseRbAr , poseRbAr
        poseLoc = PoseWithCovarianceStamped() 
        dem = 0 
        for dem in range(i):
            while (enable_poseRbAr == False): 
                dem = dem
            else:     
                poseLoc.pose.pose.position.x    = poseLoc.pose.pose.position.x    + poseRbAr.pose.pose.position.x    
                poseLoc.pose.pose.position.y    = poseLoc.pose.pose.position.y    + poseRbAr.pose.pose.position.y    
                poseLoc.pose.pose.orientation.z = poseLoc.pose.pose.orientation.z + poseRbAr.pose.pose.orientation.z 
                poseLoc.pose.pose.orientation.w = poseLoc.pose.pose.orientation.w + poseRbAr.pose.pose.orientation.w 

                dem = dem + 1
                enable_poseRbAr = False
                # print "dem= " , dem 
                # print "x= " ,poseRbAr.pose.pose.position.x   
                # print "y= " ,poseRbAr.pose.pose.position.y   
                # print "z= " ,poseRbAr.pose.pose.orientation.z
                # print "w= " ,poseRbAr.pose.pose.orientation.w
                           
        poseLoc.pose.pose.position.x    = poseLoc.pose.pose.position.x    / i
        poseLoc.pose.pose.position.y    = poseLoc.pose.pose.position.y    / i
        poseLoc.pose.pose.orientation.z = poseLoc.pose.pose.orientation.z / i
        poseLoc.pose.pose.orientation.w = poseLoc.pose.pose.orientation.w / i

        return poseLoc

    def callSup2(self, vel):
        global vel_0 
        if ( vel.angular_z == 0 and vel.linear_x ==0 ): vel_0 = True
        else : vel_0 = False

    def callSup3(self, pose):
        global poseRbMa
        poseRbMa = pose 
        enable_poseRbMa = True
        # print "\nx= " ,poseRbMa.position.x   
        # print "y= " ,poseRbMa.position.y   
        # print "z= " ,poseRbMa.orientation.z
        # print "w= " ,poseRbMa.orientation.w
        
    def send_goal(self, g):
        goal = MoveBaseGoal()
        goal = g
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(goal)
 
    def SendInitialPose(self,InitialPose, initial_pose, time_stamp):
        # goal: [x, y, yaw]
        InitialPoseMsg = PoseWithCovarianceStamped()
        InitialPoseMsg = initial_pose
        InitialPoseMsg.header.stamp = time_stamp
        InitialPoseMsg.header.frame_id = 'map'

        InitialPose.publish(InitialPoseMsg)

    def navi1_1(self):
        self.client1 = dynamic_reconfigure.client.Client("move_base", timeout=30)
        self.client1.update_configuration({"base_global_planner":"stiplanner/StiPlanner",\
                                            "base_local_planner" :"dwa_local_planner/DWAPlannerROS",\
                                            "planner_frequency":5,\
                                            "recovery_behavior_enabled":"false",\
                                            "oscillation_distance":0,\
                                            "oscillation_timeout": 10,\
                                            "max_planning_retries":10})

        self.client2 = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS", timeout=30)
        self.client2.update_configuration({"xy_goal_tolerance":0.1,\
                                            "yaw_goal_tolerance" :0.1,\
                                            "max_vel_x":0.6,\
                                            "max_vel_theta":0.5})
    
    def navi1_2(self):  
        self.client1 = dynamic_reconfigure.client.Client("move_base", timeout=30)
        self.client1.update_configuration({"base_global_planner":"stiplanner/StiPlanner",\
                                            "base_local_planner" :"dwa_local_planner/DWAPlannerROS",\
                                            "planner_frequency":5,\
                                            "recovery_behavior_enabled":"false",\
                                            "oscillation_distance":0,\
                                            "oscillation_timeout": 10,\
                                            "max_planning_retries":10})

        self.client2 = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS", timeout=30)
        self.client2.update_configuration({"xy_goal_tolerance":0.05,\
                                            "yaw_goal_tolerance" :0.1,\
                                            "max_vel_x":0.6,\
                                            "max_vel_theta":0.5})

    def ss_goal(sele,pose1, goal2):
        if ( fabs(pose1.position.x) - fabs(goal2.target_pose.pose.position.x) ) < 0.15 and \
           ( fabs(pose1.position.y) - fabs(goal2.target_pose.pose.position.y) ) < 0.15 :
           return True 
        else : return False

    def callSup4(self,data):
        global mb_read
        mb_read = data
        self.SS_BT1_1 = self.chuyendoi(mb_read.Sie_in,1) 
        self.SS_BT1_2 = self.chuyendoi(mb_read.Sie_in,2) 
        self.SS_BT2_1 = self.chuyendoi(mb_read.Sie_in,4) 
        self.SS_BT2_2 = self.chuyendoi(mb_read.Sie_in,3) 
        self.BT_EN    = self.chuyendoi(mb_read.Sie_in,7) 
        self.SS_ngoai = self.chuyendoi(mb_read.BT_SIE,8)

    def chuyendoi(self, data, vitri):
        a = 0b0000000000000001
        a = a << vitri 
        if ( data & a ) > 1 : return True
        else : return False

    def bangtai(self):
        global mb_control
        global mb_read , toyo_in, toyo_out
        print self.SS_BT1_1 ,self.SS_BT1_2 ,self.SS_BT2_1,self.SS_BT2_2,self.BT_EN ,self.SS_ngoai
        dem  = 1 
        if dem == 1 : 
            if self.BT_EN == False : # bt dang hoat dong
                mb_control.BT_SIE_OUT = int(self.bt1_in)
                mb_control.BN_FC = 2 # nang
                mb_control.BT_DC_1 = 2 # q.trai
                self.pub4.publish(mb_control)
                while self.SS_BT1_2 == True : 
                    print " wait1 " , self.SS_BT1_1
                    pass 
                else : 
                    mb_control.BT_SIE_OUT = int(self.bt_stop)
                    mb_control.BT_DC_1 = 0 # stop
                    self.pub4.publish(mb_control)
                    # rospy.sleep(1)
                    dem = 2
        if dem == 2 :
            mb_control.BT_SIE_OUT = int(self.bt1_out)
            mb_control.BT_DC_1 = 1 # q.phai
            self.pub4.publish(mb_control)
            while self.SS_ngoai == True : 
                print " wait2 " , self.SS_ngoai
                pass 
            else : 
                mb_control.BT_SIE_OUT = int(self.bt_stop)
                mb_control.BT_DC_1 = 0 # stop
                self.pub4.publish(mb_control)
                # rospy.sleep(1)
                dem = 3
        
        if dem == 3 : 
            print "hihi"
            return True

    def ledcoi(self, led, coi):
        mb_control.Led_FC = led # 2 : xanh la 1s  , 3 :xanh lam , 0.1s 
        mb_control.COI_FC = coi # 1 bao dong , 2 an toan
        self.pub4.publish(mb_control)

    def run(self):
        print('Thread #%s started' % self.ident)
       # 
        goal2_vao = MoveBaseGoal()
        goal2_bt = MoveBaseGoal()
        goal2_q = MoveBaseGoal()
        goal4_vao = MoveBaseGoal()
        goal4_bt = MoveBaseGoal()
        goal4_q = MoveBaseGoal()
        p1_th = MoveBaseGoal()
        p1_ng = MoveBaseGoal()
        p2_th = MoveBaseGoal()
        p2_ng = MoveBaseGoal()
        p3_th = MoveBaseGoal()

        dem = int()
        global enable_poseRbAr , poseRbAr , vel_0 

       #goal2_vao

        goal2_vao.target_pose.header.frame_id    = "map"  
        goal2_vao.target_pose.pose.position.x    = 0.628
        goal2_vao.target_pose.pose.position.y    = 0.088
        goal2_vao.target_pose.pose.orientation.z = 0.000
        goal2_vao.target_pose.pose.orientation.w = 1.000

       #goal2_q
        goal2_q.target_pose.header.frame_id    = "map"
        goal2_q.target_pose.pose.position.x    = 0.313
        goal2_q.target_pose.pose.position.y    = 0.059
        goal2_q.target_pose.pose.orientation.z = 0.953
        goal2_q.target_pose.pose.orientation.w = -0.302

       #goal4_vao
        goal4_vao.target_pose.header.frame_id    = "map"
        goal4_vao.target_pose.pose.position.x    = -11.707
        goal4_vao.target_pose.pose.position.y    = 1.613
        goal4_vao.target_pose.pose.orientation.z = 0.691
        goal4_vao.target_pose.pose.orientation.w = 0.723
    
       #goal4_q
        goal4_q.target_pose.header.frame_id    = "map"
        goal4_q.target_pose.pose.position.x    = -11.710
        goal4_q.target_pose.pose.position.y    = 1.461
        goal4_q.target_pose.pose.orientation.z = -0.694
        goal4_q.target_pose.pose.orientation.w = 0.720

       #p1_th
        p1_th.target_pose.header.frame_id    = "map"
        p1_th.target_pose.pose.position.x    = -0.622
        p1_th.target_pose.pose.position.y    = -0.635
        p1_th.target_pose.pose.orientation.z = 1.000
        p1_th.target_pose.pose.orientation.w = -0.000
       #p1_ng
        p1_ng.target_pose.header.frame_id    = "map"
        p1_ng.target_pose.pose.position.x    = -0.643
        p1_ng.target_pose.pose.position.y    = -0.651
        p1_ng.target_pose.pose.orientation.z = 0.321
        p1_ng.target_pose.pose.orientation.w = 0.947
       #p2_th
        p2_th.target_pose.header.frame_id    = "map"
        p2_th.target_pose.pose.position.x    = -11.477
        p2_th.target_pose.pose.position.y    = -0.774
        p2_th.target_pose.pose.orientation.z = 0.689
        p2_th.target_pose.pose.orientation.w = 0.725
       #p2_ng
        p2_ng.target_pose.header.frame_id    = "map"
        p2_ng.target_pose.pose.position.x    = -11.441
        p2_ng.target_pose.pose.position.y    = -0.792
        p2_ng.target_pose.pose.orientation.z = 1.000
        p1_ng.target_pose.pose.orientation.w = 0.00

       #
        dem = 1
        poseSet = PoseWithCovarianceStamped() 
        self.navi1_1()
        self.ledcoi(2,2)
        while not self.shutdown_flag.is_set():
            print "quytrinh_test= ", dem
            if dem == -2 :
                self.call_parking(11,0.03,0.5)
                print self.park_info
                if self.park_info != -1 :
                    dem = -1

            
            if dem == -1 :
                self.call_parking(0,0.03,0.5)
                if self.park_info == 0 :
                    dem = 0 
            

            if dem == 1 :
                self.ledcoi(2,2)
                self.navi1_2()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"              
                self.send_goal(goal2_vao)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED and \
                   self.ss_goal(poseRbMa,goal2_vao) == True :
                    dem = 2
                else: 
                    dem = 1
                    print " error 1 "
            

            if dem == 2 :
                self.ledcoi(3,2)
                while vel_0 != True : print " vel = 0 ..."
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"

                for i in xrange(2):
                    self.ReserMap.publish("reset")
                    rospy.sleep(0.01)
                print "reset map OK"

                # setpose
                poseSet= self.locPoseRb(5)
                self.SendInitialPose(self.initpose_publisher,poseSet,rospy.Time.now())

                time.sleep(0.01)
                print "Set pose OK "
                dem = 3
            
            
            if dem == 3 :
                self.call_parking(11,0.03,0.46)
                print self.park_info
                if self.park_info != -1 :
                    dem = 4

            
            if dem == 4 :
                if self.park_info == 11 :
                    self.call_parking(0,0,0)
                    rospy.sleep(1)
                    self.bangtai()
                    rospy.sleep(1)
                    dem = 5
                
            
            if dem == 5 :
                self.navi1_1()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(goal2_q)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED and \
                   self.ss_goal(poseRbMa,goal2_q) == True :
                    dem = 6 #8
                else: 
                    dem =  5
                    print " error 5"

            
            if dem == 6 :
                self.ledcoi(2,2)
                self.navi1_1()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(p1_th)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED and \
                   self.ss_goal(poseRbMa,p1_ng) == True :
                    dem = 7
                else: 
                    dem = 6
                    print " error 6 "

            
            if dem == 7 :
                self.navi1_1()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(p2_th)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED and \
                   self.ss_goal(poseRbMa,p2_th) == True :
                    dem = 8
                else: 
                    dem = 7
                    print " error 7 "

            
            if dem == 8 :
                self.navi1_2()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(goal4_vao)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED and \
                   self.ss_goal(poseRbMa,goal4_vao) == True :
                    dem = 9
                else: 
                    dem = 8
                    print " error 8 "
  
            if dem == 9 :
                self.ledcoi(3,2)
                while vel_0 != True : print " vel = 0 ..."
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"

                for i in xrange(2):
                    self.ReserMap.publish("reset")
                    rospy.sleep(0.01)
                print "reset map OK"

                # setpose
                poseSet= self.locPoseRb(5)
                self.SendInitialPose(self.initpose_publisher,poseSet,rospy.Time.now())

                time.sleep(0.01)
                print "Set pose OK "
                dem = 10
        
            
            if dem == 10 :
                self.call_parking(10,0.03,0.46)
                print self.park_info
                if self.park_info != -1 :
                    dem = 11

            
            if dem == 11 :
                if self.park_info == 11 :
                    self.call_parking(0,0,0)
                    rospy.sleep(1)
                    self.bangtai()
                    rospy.sleep(1)
                    dem = 12
                

            
            if dem == 12 :
                self.navi1_1()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(goal4_q)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED and \
                   self.ss_goal(poseRbMa,goal4_q) == True :
                    dem = 13 #1
                else: 
                    dem = 12
                    print " error 12"

            
            if dem == 13 :
                self.ledcoi(2,2)
                self.navi1_1()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(p2_ng)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED and \
                   self.ss_goal(poseRbMa,p2_ng) == True :
                    dem = 14
                else: 
                    dem = 13
                    print " error 13 "

            
            if dem == 14 :
                self.navi1_1()
                self.clear_costmaps_srv(EmptyRequest())
                print "clear_costmaps OK"
                self.send_goal(p1_ng)
                time.sleep(0.5)
                self.client.wait_for_result()
                if self.client.get_state() == GoalStatus.SUCCEEDED and \
                   self.ss_goal(poseRbMa,p1_ng) == True :
                    dem = 1
                else: 
                    dem = 14
                    print " error 14 "

            time.sleep(0.01)

        print('Thread #%s stopped' % self.ident)

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
        thread1 = quyTrinh(1,"quyTrinh")
        thread1.start()
 
        # Keep the main thread running, otherwise signals are ignored.
        while True:
            time.sleep(0.01)
 
    except ServiceExit:
        # Terminate the running threads.
        # Set the shutdown flag on each thread to trigger a clean shutdown of each thread.
        thread1.shutdown_flag.set()
        # Wait for the threads to close...
        thread1.join()
    print('Exiting main program')
 
 
if __name__ == '__main__':
    main()





    