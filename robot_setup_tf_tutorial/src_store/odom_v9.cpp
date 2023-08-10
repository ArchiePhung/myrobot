
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#define can2_2 0.7071067812
#define pi 3.141592653589793238462643383279502884

nav_msgs::Odometry odom_robot;

double oz,ow,z,w ;
double roll, pitch, yaw_ht , yaw_tr , vth_robot  ;
double dt_ht, dt_tr , deta_dt ; //s

void getOdom_t265(const nav_msgs::Odometry& odom)
{   
    dt_ht = odom.header.stamp.nsec ;
    z = odom.pose.pose.orientation.z;
    w = odom.pose.pose.orientation.w;

    deta_dt = 1000000000ul / ( dt_ht - dt_tr )   ; //hz
    oz = can2_2 * ( z - w );
    ow = can2_2 * ( z + w );

    tf::Quaternion q1(0, 0, oz, ow);
    tf::Matrix3x3 m(q1);
    m.getRPY(roll, pitch, yaw_ht); // rad
    vth_robot = deta_dt * ( yaw_ht - yaw_tr );

    // update 
    dt_tr = dt_ht ;
    yaw_tr = yaw_ht ;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "vector_t265");
  ros::NodeHandle n;
  ros::Publisher odom_pub2 = n.advertise<nav_msgs::Odometry>("odom_robot", 100);
  ros::Subscriber sub = n.subscribe("/t265/odom/sample", 100, getOdom_t265);

  tf::TransformBroadcaster odom_broadcaster6;
  ros::Time current_time, last_time ;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate r(200.0);
  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    // ROS_INFO("yaw= %f", yaw_ht );
    // aruco --> robot

    geometry_msgs::TransformStamped odom_trans6;
    odom_trans6.header.stamp = current_time;
    odom_trans6.header.frame_id = "map_ao";
    odom_trans6.child_frame_id = "rb";

    odom_trans6.transform.rotation.x = 0;
    odom_trans6.transform.rotation.y = 0;
    odom_trans6.transform.rotation.z = oz ;
    odom_trans6.transform.rotation.w = ow ;

    // odom_broadcaster6.sendTransform(odom_trans6);

    // publish odom
    odom_robot.header.stamp = current_time;
    odom_robot.header.frame_id = "odom";
    odom_robot.child_frame_id = "base_footprint";

    odom_robot.pose.pose.orientation.x = 0 ;
    odom_robot.pose.pose.orientation.y = 0 ;
    odom_robot.pose.pose.orientation.z = oz ;
    odom_robot.pose.pose.orientation.w = ow ;

    odom_robot.twist.twist.angular.z = vth_robot ;

    odom_robot.pose.covariance[0] = 0.001;
    odom_robot.pose.covariance[7] = 0.001;
    odom_robot.pose.covariance[35] = 0.001;
    odom_robot.twist.covariance[0] = 0.0001;
    odom_robot.twist.covariance[7] = 0.0001;
    odom_robot.twist.covariance[35] = 0.0001;

    //publish the message
    odom_pub2.publish(odom_robot);
    r.sleep();
     
  }
}
