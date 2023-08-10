#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <time.h>

ros::Subscriber sub ;
ros::Publisher odom_pub ;
nav_msgs::Odometry odom;

void getOdom_t265(const nav_msgs::Odometry& odom_t265)
{  
    // ros::Time current_time = ros::Time::now();

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_t265.pose.pose.orientation.x);
    
    //publish the odometry message over ROS
    odom.header.stamp = ros::Time::now();
    
    //set the position origin
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat ;

    //set the velocity
    odom.twist.twist.linear.x = -odom_t265.twist.twist.linear.z;
    // odom.twist.twist.linear.y = -odom_t265.twist.twist.linear.x;
    odom.twist.twist.angular.z = odom_t265.twist.twist.angular.x;

    //publish the message
    odom_pub.publish(odom);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_listener");
    ros::NodeHandle n ;

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    sub = n.subscribe("/t265/odom/sample", 50, getOdom_t265);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom_robot", 50);

    ros::spin();
    return 0;
}


