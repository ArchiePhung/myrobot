
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#define pi 3.141592653589793238462643383279502884

nav_msgs::Odometry odom;
nav_msgs::Odometry odom2;

float px_robot,py_robot,goc_robot ; 
double roll,roll_ht,roll_set, pitch, yaw, yaw_ht,yaw_set, yaw_rad;

void diy_tranform(){

  tf::Quaternion q1(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w);
  tf::Matrix3x3 m(q1);
  m.getRPY(roll, pitch, yaw_ht); // rad
  
 
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher odom_pub2 = n.advertise<nav_msgs::Odometry>("odom2", 50);
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster odom_broadcaster2;

  double x = 0;
  double y = 0;
  double th = 0.0;

  double vx = 0;
  double vy = -0.04;
  double vth = 0.2;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);

  ros::Time t1,t2 ;
  int buoc=0 ;
  t1=ros::Time::now();
  double d = 0.2 ; // met 

  double px_ht, px_tr , py_ht, py_tr , th_ht, th_tr ;
  double vx_robot , vy_robot , vth_robot ;

  while(n.ok()){

    diy_tranform();
    // ROS_INFO("yaw_ht= %f",yaw_ht*(180/pi));
    yaw_rad = yaw_ht; // rad
    yaw = yaw_ht*(180/pi); // do
    // if(yaw  >=-90 && yaw  <= 90){           
    //     px_robot = odom.pose.pose.position.x - d*sin(yaw_rad);
    //     py_robot = odom.pose.pose.position.y - d*cos(pi-yaw_rad);
    //     ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(1)= %f)",px_robot,py_robot,yaw);
    // }
    // else{           
    //     px_robot = odom.pose.pose.position.x - d*sin(yaw_rad);
    //     py_robot = odom.pose.pose.position.y + d*cos(yaw_rad);
    //     ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(1)= %f)",px_robot,py_robot,yaw);
    // }
    
    // if(yaw  >=0 && yaw  <= 90){           
        px_robot = odom.pose.pose.position.x + d*cos(yaw_rad);
        py_robot = odom.pose.pose.position.y + d*sin(yaw_rad);
        ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(1)= %f)",px_robot,py_robot,yaw);
    // }
    // else{           
    //     px_robot = odom.pose.pose.position.x - d*sin(yaw_rad);
    //     py_robot = odom.pose.pose.position.y + d*cos(yaw_rad);
    //     ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(1)= %f)",px_robot,py_robot,yaw);
    // }
    
    current_time = ros::Time::now();
    // tinh vx_robot,vy_robot, vth_robot
    px_ht = px_robot ;
    py_ht = py_robot ;
    th_ht = yaw_ht ;

    vx_robot = px_ht - px_tr ;
    vy_robot = py_ht - py_tr ;
    vth_robot = th_ht - th_tr ;

    
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    
/*t265 ao*/
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

 /*robot ao*/ 
     //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(yaw_rad);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans2;
    odom_trans2.header.stamp = current_time;
    odom_trans2.header.frame_id = "odom";
    odom_trans2.child_frame_id = "robot_link";

    odom_trans2.transform.translation.x = px_robot;
    odom_trans2.transform.translation.y = py_robot;
    odom_trans2.transform.translation.z = 0.0;
    odom_trans2.transform.rotation = odom_quat2;

    //send the transform
    odom_broadcaster2.sendTransform(odom_trans2);
/*
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
     tf2::Quaternion q;
    transformStamped.header.stamp = current_time;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0;
    q.setRPY(0, 0, th);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
  
    tfb.sendTransform(transformStamped);
*/   

/*t265 ao*/
    //next, we'll publish the odometry message over ROS
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation = transformStamped.transform.rotation;
    odom.pose.pose.orientation = odom_quat ;

    //set the velocity
    odom.twist.twist.linear.x = 0; //vx;
    odom.twist.twist.linear.y = 0; //vy;
    odom.twist.twist.angular.z = vth;

    ROS_INFO("t265 : x= %f y= %f ori_z= %f",x,y,odom.pose.pose.orientation.z*180);

    //publish the message
    odom_pub.publish(odom);

 /*robot ao*/ 
    //next, we'll publish the odometry message over ROS
    odom2.header.stamp = current_time;
    odom2.header.frame_id = "odom";
    odom2.child_frame_id = "robot_link";

    //set the position
    odom2.pose.pose.position.x = px_robot;
    odom2.pose.pose.position.y = py_robot;
    odom2.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation = transformStamped.transform.rotation;
    odom2.pose.pose.orientation = odom_quat2 ;

    //set the velocity
    odom2.twist.twist.linear.x = 0 ; // vx_robot;
    odom2.twist.twist.linear.y = 0 ; //vy_robot;
    odom2.twist.twist.angular.z = vth_robot;

    // ROS_INFO("robot : x= %f y= %f ori_z= %f",px_robot,py_robot,);
    ROS_INFO("v2");
    //publish the message
    odom_pub2.publish(odom2);
    
    
    last_time = current_time;
    px_tr =  px_ht ;
    py_tr =  py_ht ;
    th_tr =  th_ht ;

    r.sleep();
  }
}
