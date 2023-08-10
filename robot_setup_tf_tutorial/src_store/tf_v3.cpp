#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
// #include <math.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <time.h>
nav_msgs::Odometry odom_t265 ;
tf2::Quaternion q_orig ;
geometry_msgs::PoseStamped _pose_robot;
geometry_msgs::PoseStamped _pose_haha;

void getOdom_t265(const nav_msgs::Odometry& odom)
{  
  odom_t265 = odom ;
}

#define pi 3.141592653589793238462643383279502884

float px_t265,py_t265,oz_t265 ; 
float px_robot,py_robot,goc_robot ; 
double roll,roll_ht,roll_set, pitch, yaw, yaw_ht,yaw_set, yaw_rad;

void diy_tranform(){

  tf::Quaternion q1(odom_t265.pose.pose.orientation.x,odom_t265.pose.pose.orientation.y,odom_t265.pose.pose.orientation.z,odom_t265.pose.pose.orientation.w);
  tf::Matrix3x3 m(q1);
  m.getRPY(roll, pitch, yaw_ht); // rad
  
 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_listener");
    ros::NodeHandle n ;
    ros::Subscriber sub = n.subscribe("/t265/odom/sample", 100, getOdom_t265);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_robot", 50);
    nav_msgs::Odometry odom;
    tf2::Quaternion odom_quat;

    tf::TransformBroadcaster odom_broadcaster;
    ros::Rate loop_rate(300);
    
    double d = 0.205 ; // met 

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q;


    ros::Time t1,t2 ;
    int buoc=0 ;
    t1=ros::Time::now();

    ros::Time current_time, last_time, last_vel_time_;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    double x_ht = -0.205 ,  x_tr = -0.205 ; //px_robot  ;
    double y_ht = 0,        y_tr = 0 ;            //py_robot  ;
    double goc_ht = 0 ,     goc_tr = 0 ;

  while (ros::ok()){
    // dinh dang frame 
    if(buoc == 0){ 
      t2=ros::Time::now();
      diy_tranform();   
      yaw_set = 0 ;
      roll_set = 0 ;
      if( t2.toSec()-t1.toSec() > 5){
        yaw_set = yaw_ht ;
        yaw_set = (180/pi)*yaw_set;
        roll_set = roll_ht ;
        buoc = 1 ;
      } 
 
    }

    if( buoc == 1){

        // chuyen doi toa do 
        diy_tranform();

        yaw     = (180/pi)*yaw ;
        // yaw_set = (180/pi)*yaw_set ;
        yaw_ht  = (180/pi)*yaw_ht ;

        if(yaw_set >=0 ){
            
            if(yaw_ht >= -180 && yaw_ht <=  yaw_set -180  ){
                 yaw =  (180 - yaw_set) + (180 + yaw_ht) ;
            }
            else{
                yaw =  yaw_ht - yaw_set;
           }
        }

        if(yaw_set <0 ){
            
            if(yaw_ht <= 180 && yaw_ht >=  yaw_set + 180  ){
                 yaw =  (180 + yaw_set) + (180 - yaw_ht) ;
            }
            else{
                yaw =  yaw_ht - yaw_set;
           }
        }


        yaw_rad = yaw*(pi/180);

        if(yaw  >=0 && yaw  <= 90){           
            px_robot = odom_t265.pose.pose.position.x - d*cos(yaw_rad);
            py_robot = odom_t265.pose.pose.position.y - d*sin(yaw_rad);
            ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(1)= %f yaw_rad= %f )\n ",px_robot,py_robot,yaw,yaw_rad );
        }
        if(yaw  >90 && yaw  <= 180){
            px_robot = odom_t265.pose.pose.position.x + d*cos(180 - yaw_rad);
            py_robot = odom_t265.pose.pose.position.y - d*sin(180 - yaw_rad);
            ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(2)= %f) \n ",px_robot,py_robot,yaw );
        }
        if(yaw  >=-180 && yaw  <= -90){
            px_robot = odom_t265.pose.pose.position.x + d*cos(-180 -yaw_rad);
            py_robot = odom_t265.pose.pose.position.y - d*sin(-180 -yaw_rad);
            ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(3)= %f) \n ",px_robot,py_robot,yaw );
        }
        if(yaw  >-90 && yaw  <0){
            px_robot = odom_t265.pose.pose.position.x - d*cos(yaw_rad);
            py_robot = odom_t265.pose.pose.position.y - d*sin(yaw_rad);
            ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(4)= %f) \n ",px_robot,py_robot,yaw );
        }
    
   
//--------------- publish odom robot------------------------------------// 
        ROS_INFO("-- D28 --");

        /*
          current_time = ros::Time::now();
          double dt = (current_time - last_time).toSec();
          x_ht = px_robot ;
          y_ht = py_robot ; 
          goc_ht = yaw_rad ;

          double vx = (( x_ht- x_tr )/ dt) ; // m/s 
          double vy = (( y_ht- y_tr )/ dt) ; // m/s 
          double v_yaw = (( goc_ht- goc_tr )/ dt) ; // rad/s 

          ROS_INFO("vx = %f vy = %f v_yaw = %f \n",vx,vy,v_yaw);
        */

        // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw_rad);
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_t265.pose.pose.orientation.z);
        

        //publish the odometry message over ROS
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        //set the position origin
        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat ;

        //set the velocity
        odom.twist.twist.linear.x = -odom_t265.twist.twist.linear.z;
        // odom.twist.twist.linear.y = -odom_t265.twist.twist.linear.x;
        odom.twist.twist.angular.z = odom_t265.twist.twist.angular.x;

        //  odom.twist.twist.linear.x =vx;
        // odom.twist.twist.linear.y = vy;
        // odom.twist.twist.angular.z = v_yaw;

        //publish the message
        odom_pub.publish(odom);
        
    }
    // last_time = current_time ;
    // x_tr = x_ht  ;
    // y_tr = y_ht  ;
    // goc_tr = goc_ht ;

    loop_rate.sleep();
    ros::spinOnce(); 

  }
}