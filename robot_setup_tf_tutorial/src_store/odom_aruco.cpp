#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#define pi 3.141592653589793238462643383279502884

nav_msgs::Odometry odom_t265_ao ,odom_robot ,odom_t265 ;

float px_robot,py_robot,goc_robot ; 
double roll,roll_ht,roll_set, pitch, yaw, yaw_ht,yaw_set, yaw_rad;

// double d = -0.205 ; // met 
double d = 0.26 ; // met

double px_ht, px_tr ,deta_px , py_ht, py_tr ,deta_py, th_ht, th_tr,deta_th ;
double vx_robot , vy_robot , vth_robot ;

double px_ht_t265, px_tr_t265 , py_ht_t265, py_tr_t265 , th_ht_t265, th_tr_t265 ;
double vx_t265 , vy_t265 , vth_t265 ;

double dt_ht, dt_tr , deta_dt ; //s


void getOdom_t265(const nav_msgs::Odometry& odom_hihi)
{   
    dt_ht = odom_hihi.header.stamp.nsec ;
    // deta_dt = ( dt_ht - dt_tr ) / 1000000000ul ;
    deta_dt = 1000000000ul / ( dt_ht - dt_tr )   ;

    tf::Quaternion q1(odom_hihi.pose.pose.orientation.x,odom_hihi.pose.pose.orientation.y,odom_hihi.pose.pose.orientation.z,odom_hihi.pose.pose.orientation.w);
    tf::Matrix3x3 m(q1);
    m.getRPY(roll, pitch, yaw_ht); // rad
    yaw_rad = yaw_ht ; // rad
    yaw = (180/pi)*yaw_ht ; // do

    // tinh toan cho t265
    px_ht_t265 = d + odom_hihi.pose.pose.position.x ;
    py_ht_t265 = odom_hihi.pose.pose.position.y ;
    th_ht_t265 = yaw_rad ;

    // tinh vx_robot,vy_robot, vth_robot
    px_robot = px_ht_t265 - d*cos(yaw_rad);
    py_robot = py_ht_t265 - d*sin(yaw_rad);
    
    px_ht = px_robot ;
    py_ht = py_robot ;
    th_ht = yaw_rad ;
    
    vx_robot = deta_dt * ( px_ht - px_tr );
    vy_robot = deta_dt * ( py_ht - py_tr ) ;
    
    vth_robot = deta_dt * ( th_ht - th_tr );
    // vth_robot =(1/deta_dt)*( th_ht - th_tr );
    // vth_robot = ( th_ht - th_tr );
 
     // update 
    dt_tr = dt_ht ;
    px_tr =  px_ht ;
    py_tr =  py_ht ;
    th_tr =  th_ht ;

    px_tr_t265 =  px_ht_t265 ;
    py_tr_t265 =  py_ht_t265 ;
    th_tr_t265 =  th_ht_t265 ;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/t265/odom/sample", 100, getOdom_t265);
  ros::Publisher odom_pub2 = n.advertise<nav_msgs::Odometry>("odom_robot", 50);
  tf::TransformBroadcaster odom_broadcaster2;
  ros::Time current_time, last_time ;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate r(200.0);

  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();

    ROS_INFO("yaw= %f", yaw );

    geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(yaw_rad);

    //next, we'll publish the odometry message over ROS
    odom_robot.header.stamp = current_time;
    odom_robot.header.frame_id = "odom_robot";
    odom_robot.child_frame_id = "base_robot";

    //set the position
    odom_robot.pose.pose.position.x = px_robot;
    odom_robot.pose.pose.position.y = py_robot;
    odom_robot.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation = transformStamped.transform.rotation;
    odom_robot.pose.pose.orientation = odom_quat2 ;

    //set the velocity
    odom_robot.twist.twist.linear.x = vx_robot;
    odom_robot.twist.twist.linear.y = vy_robot;
    odom_robot.twist.twist.angular.z = vth_robot ;

    odom_robot.pose.covariance[0] = 0.001;
    odom_robot.pose.covariance[7] = 0.001;
    odom_robot.pose.covariance[35] = 0.001;
    odom_robot.twist.covariance[0] = 0.0001;
    odom_robot.twist.covariance[7] = 0.0001;
    odom_robot.twist.covariance[35] = 0.0001;

    //publish the message
    odom_pub2.publish(odom_robot);

     /*robot ao*/ 

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans2;
    odom_trans2.header.stamp = current_time;
    odom_trans2.header.frame_id = "odom_robot";
    odom_trans2.child_frame_id = "base_robot";

    odom_trans2.transform.translation.x = px_robot;
    odom_trans2.transform.translation.y = py_robot;
    odom_trans2.transform.translation.z = 0.0;
    odom_trans2.transform.rotation = odom_quat2;

    //send the transform
    odom_broadcaster2.sendTransform(odom_trans2);
    
    r.sleep();
     
  }
}
