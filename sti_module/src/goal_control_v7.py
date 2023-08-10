#!/usr/bin/python
#author : AnhTuan 27/5/2020
#update 17/9/2020
#update 27/01/2020 : update cancel goal : cg
#update 2/2/2020 :  update one thread

import threading
import time
import rospy
from std_msgs.msg import String, Bool

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
from nav_msgs.msg import Odometry

from math import pi as PI
from math import atan2, sin, cos, sqrt , fabs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import dynamic_reconfigure.client
import Queue
from sti_msgs.msg import *
import time 
import os 
from nav_msgs.msg import Path

#  pose Robot from Aruco
poseRbAr = PoseWithCovarianceStamped()  
enable_poseRbAr = False
#  pose Robot in Map
poseRbMa = Pose()
vel_robot = 0 
 
class goalControl():

    def __init__(self):
        # threading.Thread.__init__(self)
        # self.threadID = threadID
        # self.shutdown_flag = threading.Event()
        rospy.init_node('goal_control', anonymous=True)

        #get param 
        self.rate = rospy.get_param('~rate',20) 
        self.vel_x_max = rospy.get_param('~vel_x_max',0.7)
        self.vel_x_min = rospy.get_param('~vel_x_min',0.1)
        self.vel_theta_max = rospy.get_param('~vel_theta_max',0.7)
        self.vel_theta_min = rospy.get_param('~vel_theta_min',0.1)
        self.tolerance_xy_max = rospy.get_param('~tolerance_xy_max',0.2)
        self.tolerance_xy_min = rospy.get_param('~tolerance_xy_min',0.02)
        self.tolerance_theta_max = rospy.get_param('~tolerance_theta_max',0.2)
        self.tolerance_theta_min = rospy.get_param('~tolerance_theta_min',0.02)
        self.planner_frequency_max = rospy.get_param('~planner_frequency_max',50)
        self.planner_frequency_min = rospy.get_param('~planner_frequency_min',0)
        self.tolerance_rot_step1 = rospy.get_param('~tolerance_rot_step1',0.05)
        self.vel_rot_step1 = rospy.get_param('~vel_rot_step1',0.5)

        self.gioihan_lui  = rospy.get_param('~gioihan_lui',2.5)

        #rate
        self.rate_thread = rospy.Rate(self.rate)
        #giao tiep
        self.pub = rospy.Publisher('/status_goal_control', Status_goalControl, queue_size=10) 
        rospy.Subscriber("/goal_control", Goal_control, self.callSup_goal_control)
        rospy.Subscriber("/posePoseRobotFromAruco", PoseWithCovarianceStamped, self.callSup_posePose)
        rospy.Subscriber("/raw_vel", Velocities, self.callSup2)
        rospy.Subscriber("/robot_pose", Pose, self.callSup3)
        rospy.Subscriber("/zone_robot", Zone_lidar_2head, self.callSup4)
        rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 1)

        # rospy.Subscriber('/cmd_vel_moveBase', Twist, self.callSup5, queue_size = 1)
        rospy.Subscriber('/raw_vel', Velocities, self.callSup5, queue_size = 1)
        

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
        #cmd_vel
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel_moveBase', Twist, queue_size=1)
        self.pub_cmd_vel_ps2 = rospy.Publisher('/cmd_vel_ps2', Twist, queue_size=1)
        #fnShutDown
        rospy.on_shutdown(self.fnShutDown)

        # plan 
        rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.callSup6, queue_size = 100)
        self.is_localPlan = False
        # vel EMC 
        self.pub_cmdVel_EMC   = rospy.Publisher('/cmd_vel_EMC', Bool, queue_size=100)
        

        #bien goal_control
        self.is_receivied_goal_control = False 
        self.goal_control = Goal_control()
        self.goal_control.send_goal = PoseStamped()
        self.goal_control.enable_move = 0
        self.goal_control.vel_x = 0.6
        self.goal_control.vel_theta = 0.5
        self.goal_control.tolerance_xy = 0.1
        self.goal_control.tolerance_theta = 0.1
        self.goal_control.planner_frequency = 5
        #bien goal status
        self.stt_gc = Status_goalControl()
        self.stt_gc.status_goal = 0
        self.stt_gc.check_received_goal = 0
        self.stt_gc.distance_goal = .0
        #dung tinh toan k/c
        self.poseRbMa = Pose()
        self.kc_backward = .0
        #bien Zone_lidar
        self.zone = Zone_lidar_2head()
        self.check_zone1 = 0
        self.vel = Twist()
        self.t_ht = rospy.get_time()
        self.t_tr = rospy.get_time()
        self.t_ht2 = rospy.get_time()
        self.t_tr2 = rospy.get_time()

        #odom 
        self.odom_x = .0
        self.odom_y = .0
        self.odom_g = .0
        #vel 
        self.rawVel = Velocities()

    def cbGetRobotOdom(self, robot_odom_msg):
        self.odom_x = robot_odom_msg.pose.pose.position.x
        self.odom_y = robot_odom_msg.pose.pose.position.y

    def pub_status(self, status_goal,check_received_goal, distance ):
        status = Status_goalControl()
        status.status_goal = status_goal
        status.check_received_goal = check_received_goal
        status.distance_goal = distance
        self.pub.publish(status)
    
    def callSup_goal_control(self,data):
        # if  self.is_receivied_goal_control == False :
        #     self.is_receivied_goal_control = True
        self.goal_control = data 
        
    def callSup_posePose(self, pose):
        global enable_poseRbAr
        global poseRbAr
        poseRbAr = pose
        enable_poseRbAr = True

    def callSup2(self, vel):
        self.rawVel = vel 

    def callSup3(self, pose):
        self.poseRbMa = pose 

    def callSup4(self, zone):
        self.zone = zone

    def callSup5(self, vel):
        self.vel = vel 
    
    def callSup6(self,plan):
        if self.is_localPlan == False :
            self.is_localPlan = True
        
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
        
    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def constrain(self,val, min_val, max_val):
        if val < min_val: return min_val
        if val > max_val: return max_val
        return val

    def navi1_2(self,vel_x,vel_theta,tolerance_xy,tolerance_theta,planner_frequency):  
        # loc gia tri
        f_vel_x = self.constrain(vel_x, self.vel_x_min, self.vel_x_max)
        f_vel_theta = self.constrain(vel_theta, self.vel_theta_min, self.vel_theta_max)
        f_tolerance_xy = self.constrain(tolerance_xy, self.tolerance_xy_min , self.tolerance_xy_max)
        f_tolerance_theta = self.constrain(tolerance_theta, self.tolerance_theta_min, self.tolerance_theta_max)
        f_planner_frequency = self.constrain(planner_frequency, self.planner_frequency_min, self.planner_frequency_max)

        self.client1 = dynamic_reconfigure.client.Client("move_base", timeout=30)
        self.client1.update_configuration({"base_global_planner":"stiplanner/StiPlanner",\
                                            "base_local_planner" :"dwa_local_planner/DWAPlannerROS",\
                                            "planner_frequency":f_planner_frequency,\
                                            "recovery_behavior_enabled":"false",\
                                            "oscillation_distance":0,\
                                            "oscillation_timeout": 10,\
                                            "max_planning_retries":10})

        self.client2 = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS", timeout=30)
        self.client2.update_configuration({"xy_goal_tolerance":f_tolerance_xy,\
                                            "yaw_goal_tolerance" :f_tolerance_theta,\
                                            "max_vel_x":f_vel_x,\
                                            "min_vel_x":-f_vel_x,\
                                            "max_vel_theta":f_vel_theta,\
                                            "min_vel_theta":-f_vel_theta})

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist()) 

    def check_goal(sele,pose1, goal2, tolerance):
        if ( fabs(pose1.position.x) - fabs(goal2.target_pose.pose.position.x) ) < tolerance and \
           ( fabs(pose1.position.y) - fabs(goal2.target_pose.pose.position.y) ) < tolerance :
           return True  
        else : return False 
    

    def raa(self):
        #goal2_vao
       
        # print('Thread #%s started' % self.ident)
        buoc = 0
        step = 0 
        odom_x_ht = 0.
        odom_y_ht = 0.

        while not rospy.is_shutdown() :

            #===== GOAL =====#
            # move with LIDAR

            if self.goal_control.enable_move == 1 :
                g_ctl_new = Goal_control()

                # nhan data
                if buoc == 0 :        
                    rospy.loginfo("buoc= %s",buoc)

                    # send check received goal
                    self.stt_gc.check_received_goal = 1 
                    rospy.loginfo("send check received goal")

                    # pub status
                    self.stt_gc.status_goal = 0
                    self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
                                    self.stt_gc.distance_goal )
                    
                    # get new goal_control
                    ori = self.goal_control.send_goal.pose.orientation 
                    buoc = 1 

                    # if ori.z == 99 and ori.w == 99 : # tinh ori 
                    #     buoc = 1 
                    # else :
                    #     buoc = 2
                    #     #get Goal
                    #     g_ctl_new = self.goal_control 
                
                #quay goc ( cho goal k co goc )
                if buoc == 1 :
                    # tinh goc robot hien tai
                    quata = ( self.poseRbMa.orientation.x,\
                            self.poseRbMa.orientation.y,\
                            self.poseRbMa.orientation.z,\
                            self.poseRbMa.orientation.w )
                    euler = euler_from_quaternion(quata)
                    theta_rb_ht = euler[2]
                    # theta_rb_ht = theta_rb_ht*180 / PI

                    # tinh goc poin to poin 
                    theta_poin = atan2(self.goal_control.send_goal.pose.position.y - self.poseRbMa.position.y, \
                                       self.goal_control.send_goal.pose.position.x - self.poseRbMa.position.x )
                    # theta_poin  = theta_poin*180 / PI 

                    # goc robot so voi target 
                    theta = theta_poin - theta_rb_ht
                    print "theta_rb_ht: %f - theta_poin: %f - theta: %f" %(theta_rb_ht,theta_poin,theta)
					
					# pub status : TH base khong chay
                    self.stt_gc.status_goal = 1 
                    self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
                                    self.stt_gc.distance_goal )

                    # quay
                    if fabs(theta) > self.tolerance_rot_step1 : # +- 10 do
                        if (theta > PI) or (theta > -PI and theta < 0 ): #quay phai , vel_z < 0
                            print "a"
                            twist = Twist()
                            twist.angular.z = -fabs(theta) - 0.02
                            if twist.angular.z < -self.vel_rot_step1 : twist.angular.z = -self.vel_rot_step1
                            self.pub_cmd_vel.publish(twist)
                            # buoc = 1

                        if (theta < -PI) or (theta < PI and theta > 0 ): #quay trai , vel_z > 0
                            print "b"
                            twist = Twist()
                            twist.angular.z = fabs(theta) + 0.02
                            if twist.angular.z > self.vel_rot_step1 : twist.angular.z = self.vel_rot_step1 
                            self.pub_cmd_vel.publish(twist)
                            # buoc = 1

                    else : 
                        self.pub_cmd_vel.publish(Twist())
                        buoc = 2 

                if buoc == 2:
                    if ori.z == 99 and ori.w == 99 : 
                        g_ctl_new = self.goal_control
                        g_ctl_new.send_goal.pose.orientation = self.poseRbMa.orientation 
                    else :
                        g_ctl_new = self.goal_control 
                    buoc = 3
                    
                # send goal
                if buoc == 3 :
                    # reset cmd vel EMC
                    self.pub_cmdVel_EMC.publish(False)

                    #clear costmap
                    self.clear_costmaps_srv(EmptyRequest())
                    rospy.loginfo("clear costmap")
                    rospy.sleep(1)

                    #set param navi
                    self.navi1_2(g_ctl_new.vel_x, \
                                g_ctl_new.vel_theta, \
                                g_ctl_new.tolerance_xy, \
                                g_ctl_new.tolerance_theta, \
                                g_ctl_new.planner_frequency)
                    rospy.loginfo("set param navi")

                    # send goal
                    goal_new = MoveBaseGoal(g_ctl_new.send_goal)
                    for i in range(2):                      
                        self.send_goal(goal_new)
                        rospy.loginfo("send goal")

                    # pub status
                    self.stt_gc.status_goal = 1 
                    self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
                                    self.stt_gc.distance_goal )
                    buoc = 4
                    rospy.loginfo("pub status , buoc = 3")
                    self.t_tr = self.t_ht = rospy.get_time()
                    self.t_tr2 = self.t_ht2 = rospy.get_time()
                
                # get status, check goal , check object
                if buoc == 4 :
                    rospy.loginfo("buoc= %s",buoc)

                    # get status
                    self.stt_gc.status_goal = self.client.get_state()
                    self.stt_gc.distance_goal = self.fnCalcDistPoints(  self.poseRbMa.position.x,\
                                                                        goal_new.target_pose.pose.position.x,\
                                                                        self.poseRbMa.position.y,\
                                                                        goal_new.target_pose.pose.position.y )
                       
                    #check goal                                                 
                    if self.stt_gc.status_goal == 3 :
                        if self.check_goal(self.poseRbMa,goal_new,self.goal_control.tolerance_xy) == False :
                        # if self.stt_gc.distance_goal > self.goal_control.tolerance_xy :
                            self.stt_gc.status_goal = 10 # error
                            buoc = 3 # gui lai goal 
                            rospy.logwarn("ko du k/c")
                            self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal, self.stt_gc.distance_goal )
                        else:
                            self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal, self.stt_gc.distance_goal )
                            rospy.logwarn("OK")
                    else :
                        self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal, self.stt_gc.distance_goal )

                    if self.stt_gc.status_goal == 1 :
                        self.t_ht = rospy.get_time()
                        # if self.vel.linear.x == 0 and  self.vel.angular.z == 0 : # /cmd_vel
                        if self.vel.linear_x == 0 and  self.vel.angular_z == 0 : # /raw_vel
                            # rospy.logwarn("not move time= {}".format(self.t_ht - self.t_tr))
                            if self.t_ht - self.t_tr > 5  :
                                rospy.logerr("not move -->  buoc = 3")
                                buoc = 3 
                        else :
                            self.t_tr = self.t_ht 

                    if self.is_localPlan == False :
                        self.t_ht2 = rospy.get_time()
                        rospy.logwarn("NO PLAN time= {}".format(self.t_ht2 - self.t_tr2))
                        if self.t_ht2 - self.t_tr2 > 2  :
                            if self.vel.linear_x > 0.1 :
                                rospy.logerr("NO PLAN")
                                # self.pub_cmdVel_EMC.publish(True)
                                os.system('rosrun sti_module cancelGoal.py')
                                for i in range(3):
                                    self.pub_cmd_vel.publish(Twist())
                                buoc = 3 
                                
                    else :
                        self.t_tr2 = self.t_ht2 = rospy.get_time()
                        self.is_localPlan = False
                        self.pub_cmdVel_EMC.publish(False) 
      
                    rospy.loginfo("status_goal= %s - distance_goal= %s",self.stt_gc.status_goal,self.stt_gc.distance_goal)          

            if self.goal_control.enable_move == 2 : # backward 

                if step == 0 : # nhan data       
                    rospy.loginfo("buoc= %s",buoc)

                    # send check received goal
                    self.stt_gc.check_received_goal = 1 
                    rospy.loginfo("send check received goal - LUI ")

                    # pub status
                    self.stt_gc.status_goal = 0
                    self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
                                    self.stt_gc.distance_goal )
                    
                    # get new goal_control
                    # ori = self.goal_control.send_goal.pose.orientation 
                    step = 1 
                
                if step == 1 : #tinh k/s lui
                    self.kc_backward = self.fnCalcDistPoints(  self.poseRbMa.position.x,\
                                                                self.goal_control.send_goal.pose.position.x,\
                                                                self.poseRbMa.position.y,\
                                                                self.goal_control.send_goal.pose.position.y )
                    if fabs(self.kc_backward) > self.gioihan_lui :
                        self.kc_backward = self.gioihan_lui 

                    rospy.logerr("x1= %s",self.poseRbMa.position.x)
                    rospy.logerr("x2= %s",self.goal_control.send_goal.pose.position.x)
                    rospy.logerr("y1= %s",self.poseRbMa.position.y)
                    rospy.logerr("y2= %s",self.goal_control.send_goal.pose.position.y)

                    rospy.logerr("s_targhet= %s",fabs(self.kc_backward))
                    odom_x_ht = self.odom_x
                    odom_y_ht = self.odom_y
                    # pub status
                    self.pub_status(1,1,0)
                    step = 2 

                if step == 2 : # them lui theo toa do , check /zone robot 
                    twist = Twist()
                    s = self.fnCalcDistPoints(self.odom_x,odom_x_ht,self.odom_y,odom_y_ht)
                    rospy.logwarn("s= %s",s)
                    rospy.logwarn("s_targhet= %s",fabs(self.kc_backward))
                    
                    if s < fabs(self.kc_backward) :
                        if self.zone.zone_behind == 1 :
                            twist.linear.x = 0
                        else :
                            twist.linear.x = -0.2
                        self.pub_cmd_vel.publish(twist)
                        self.pub_status(1,1,0)
                    else :
                        self.pub_cmd_vel.publish(Twist())
                        # pub status
                        self.pub_status(3,1,0)

			if self.goal_control.enable_move == -1 :
				self.stt_gc.check_received_goal = -1
				self.stt_gc.status_goal = -1
				self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
								self.stt_gc.distance_goal )
				# cancel goal                 
				# self.client.cancel_all_goals()
				# for i in range(3):
				os.system('rosrun sti_module cancelGoal.py')

				rospy.loginfo("cancel_all_goals")
				buoc = 0

            if self.goal_control.enable_move == 0 :
                rospy.loginfo("reset and wait command")
                self.stt_gc.check_received_goal = -1
                self.stt_gc.status_goal = -1
                self.pub_status(self.stt_gc.status_goal,self.stt_gc.check_received_goal,\
                                self.stt_gc.distance_goal )
                buoc = 0
                step = 0

            
            self.rate_thread.sleep()

       

    


def main():
    
    try:
        m = goalControl()
        m.raa()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main()