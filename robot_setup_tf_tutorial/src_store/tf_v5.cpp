
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry odom_t265 ;
double x_ht,y_ht,th_ht ;
double x_tr,y_tr,th_tr ;
double deta_x,deta_y,deta_th,deta_t;
// ros::Time time;
double t_sec, t_nsec,t_ht, t_tr;
double vx,vy,vth,th;

void getOdom_t265(const nav_msgs::Odometry& odom)
{  
    //get data 
    t_sec  = odom.header.stamp.sec ;      //sec += (nsec / 1000000000);
    t_nsec  = odom.header.stamp.nsec ;
    // t_ht += (t_nsec / 1000000000ul);
    t_ht = (t_nsec / 1000000000ul); // giay : s
    
    x_ht  = odom.pose.pose.position.x ;  // m
    y_ht  = odom.pose.pose.position.y ;  // m
    th_ht = odom.twist.twist.angular.x ; // rad/s

    // deta
    deta_t = t_ht - t_tr ; // s
    deta_x = x_ht - x_tr ; 
    deta_y = y_ht - y_tr ;
    deta_th = th_ht*deta_t ;

    //van toc 
    vx = deta_x / deta_t ; // m/s
    vy = deta_y / deta_t ; // m/s
    th += deta_th ;
    

    ROS_INFO("deta_x= %f deta_y= %f deta_th= %f deta_t= %f",deta_x,deta_y,deta_th,deta_t);
    ROS_INFO("vx= %f vy= %f vth= %f ",vx,vy,vth);
    ROS_INFO("v3");

    // update 
    x_tr = x_ht ;
    y_tr = y_ht ;
    th_tr = th_ht ;
    t_tr = t_ht ;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_robot", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Subscriber sub = n.subscribe("/t265/odom/sample", 100, getOdom_t265);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0;
  double vy = 0;
  double vth = 0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(250);
  while(n.ok()){

    // ROS_INFO("loop : deta_x= %f deta_y= %f deta_th= %f deta_t= %f",deta_x,deta_y,deta_th,deta_t);

    // current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    // double dt = (current_time - last_time).toSec();
    // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    // double delta_th = vth * dt;

    // x += delta_x;
    // y += delta_y;
    // th += delta_th;

    // ROS_INFO("x= %f y= %f th= %f dt= %f",x,y,th,dt);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = current_time;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "base_link";

    // odom_trans.transform.translation.x = x_ht;
    // odom_trans.transform.translation.y = x_ht;
    // odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation = odom_quat;

    // //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //set the position
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    // last_time = current_time;
    r.sleep();
    ros::spinOnce(); 
  }
}
