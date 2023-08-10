
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#define pi 3.141592653589793238462643383279502884

float px_robot,py_robot,goc_robot ; 
double roll,roll_ht,roll_set, pitch, yaw, yaw_ht,yaw_set, yaw_rad;

// double d = -0.205 ; // met 
double d = 0.3 ; // met

double px_ht, px_tr ,deta_px , py_ht, py_tr ,deta_py, th_ht, th_tr,deta_th ;
double vx_robot , vy_robot , vth_robot ;

double px_ht_t265, px_tr_t265 , py_ht_t265, py_tr_t265 , th_ht_t265, th_tr_t265 ;
double vx_t265 , vy_t265 , vth_t265 ;

double dt_ht, dt_tr , deta_dt ; //s

void getOdom_t265(const nav_msgs::Odometry& odom_t265);
void t265_ao(ros::Publisher pub_odom ,ros::Time time);
void robot_ao(ros::Publisher pub_odom ,ros::Time time);

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/t265/odom/sample", 100, getOdom_t265);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher odom_pub2 = n.advertise<nav_msgs::Odometry>("odom2", 50);

    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    ros::Rate r(200.0);

    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();

        // ROS_INFO("vx_t= %f , vy_t= %f , vth_t= %f --> vx_r= %f , vy_r= %f , vth_r= %f ",vx_t265,vy_t265,vth_t265,vx_robot,vy_robot,vth_robot);
        // ROS_INFO("yaw= %f", yaw );      
        // ROS_INFO("robot : vx_robot= %f vy_robot= %f ",vx_robot,vy_robot);
        // ROS_INFO("t265 : x= %f y= %f yaw= %f",px_ht_t265,py_ht_t265,yaw);
        // ROS_INFO("robot: x_rb= %f y_rb= %f yaw_rb(1)= %f",px_robot,py_robot,yaw);

        t265_ao(odom_pub , current_time );   
        robot_ao(odom_pub2 , current_time );

        r.sleep();    
    }
}

void getOdom_t265(const nav_msgs::Odometry& odom_t265){   
    dt_ht = odom_t265.header.stamp.nsec ;
    deta_dt = ( dt_ht - dt_tr ) / 1000000000ul ;

    // convert : quatanion --> eurle 
    tf::Quaternion q1(odom_t265.pose.pose.orientation.x,odom_t265.pose.pose.orientation.y,odom_t265.pose.pose.orientation.z,odom_t265.pose.pose.orientation.w);
    tf::Matrix3x3 m(q1);
    m.getRPY(roll, pitch, yaw_ht); // rad
    yaw_rad = yaw_ht ; // rad
    yaw = (180/pi)*yaw_ht ; // do

    // tinh toan cho t265
    px_ht_t265 = odom_t265.pose.pose.position.x ;
    py_ht_t265 = odom_t265.pose.pose.position.y ;
    th_ht_t265 = yaw_rad ;

    vx_t265 = px_ht_t265 - px_tr_t265 ;
    vy_t265 = py_ht_t265 - py_tr_t265 ;
    vth_t265 = th_ht_t265 - th_tr_t265 ;  

    // tinh vx_robot,vy_robot, vth_robot
    px_robot = px_ht_t265 - d*cos(yaw_rad);
    py_robot = py_ht_t265 - d*sin(yaw_rad);

    px_ht = px_robot ;
    py_ht = py_robot ;
    th_ht = yaw_rad ;
    
    vx_robot = px_ht - px_tr ;
    vy_robot = py_ht - py_tr ;
    // vth_robot =(1/deta_dt)*( th_ht - th_tr );
    vth_robot = ( th_ht - th_tr );
 
     // update 
    dt_tr = dt_ht ;
    px_tr =  px_ht ;
    py_tr =  py_ht ;
    th_tr =  th_ht ;

    px_tr_t265 =  px_ht_t265 ;
    py_tr_t265 =  py_ht_t265 ;
    th_tr_t265 =  th_ht_t265 ;

}

void t265_ao(ros::Publisher pub_odom ,ros::Time time){

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw_rad);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "t265_ao";

    odom_trans.transform.translation.x = px_ht_t265;
    odom_trans.transform.translation.y = py_ht_t265;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    tf::TransformBroadcaster odom_broadcaster;
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom ;
    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "t265_ao";

    //set the position
    odom.pose.pose.position.x = px_ht_t265;
    odom.pose.pose.position.y = py_ht_t265;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat ;
    //set the velocity
    odom.twist.twist.linear.x = vx_t265;
    odom.twist.twist.linear.y = vy_t265;
    odom.twist.twist.angular.z = vth_t265;   
    //publish the message
    pub_odom.publish(odom);
}

void robot_ao(ros::Publisher pub_odom ,ros::Time time){
    
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(yaw_rad);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans2;
    odom_trans2.header.stamp = time;
    odom_trans2.header.frame_id = "odom";
    odom_trans2.child_frame_id = "robot_link";

    odom_trans2.transform.translation.x = px_robot;
    odom_trans2.transform.translation.y = py_robot;
    odom_trans2.transform.translation.z = 0.0;
    odom_trans2.transform.rotation = odom_quat2;

    //send the transform
    tf::TransformBroadcaster odom_broadcaster2;
    odom_broadcaster2.sendTransform(odom_trans2);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom2;
    odom2.header.stamp = time;
    odom2.header.frame_id = "odom";
    odom2.child_frame_id = "robot_link";

    //set the position
    odom2.pose.pose.position.x = px_robot;
    odom2.pose.pose.position.y = py_robot;
    odom2.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation = transformStamped.transform.rotation;
    odom2.pose.pose.orientation = odom_quat2 ;

    //set the velocity
    odom2.twist.twist.linear.x = vx_robot;
    odom2.twist.twist.linear.y = vy_robot;
    odom2.twist.twist.angular.z = vth_robot ;
    //publish the message
    pub_odom.publish(odom2);
   
}
