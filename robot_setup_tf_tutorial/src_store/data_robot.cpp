
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

/* msg: nav_msgs/Odometry

      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      string child_frame_id
      geometry_msgs/PoseWithCovariance pose
        geometry_msgs/Pose pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        float64[36] covariance
      geometry_msgs/TwistWithCovariance twist
        geometry_msgs/Twist twist
          geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
          geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
        float64[36] covariance
*/

/* msg : geometry_msgs/PointStamped

      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      geometry_msgs/Point point
        float64 x
        float64 y
        float64 z
*/

/* msg : geometry_msgs/PoseStamped
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
*/

/*  msg : geometry_msgs/Vector3Stamped

    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Vector3 vector
      float64 x
      float64 y
      float64 z
*/

/*  msg : geometry_msgs/Quaternion

      float64 x
      float64 y
      float64 z
      float64 w
*/

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0; // = linear.z ( t265 )
  double vy = 0.0; // = 0 
  double vth = 0.0;// = -angular.x

// nav_msgs::Odometry odom_t265 ; 
// get data odomtry t265
void getOdom_t265(const nav_msgs::Odometry& odom_t265)
{  
  //odom_t265 = odom ;
  vx  = - odom_t265.twist.twist.linear.z; 
  // vy  = odom_t265.twist.twist.linear.x;
  vy= 0 ;  
  vth = odom_t265.twist.twist.angular.x;
}


int main(int argc, char** argv){
  
  ros::init(argc, argv, "robot_tf_listener");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("t265/odom/sample", 50, getOdom_t265);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry >("robot_link", 50);
  
  tf::TransformBroadcaster odom_broadcaster;

  x = y = th = vx = vy = vth = 0; 

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate loop_rate(1000);


  while (ros::ok()){
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    // ROS_INFO("tranform runing\n");

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    last_time = current_time;
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

/*  geometry_msgs/TransformStamped
    std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
    string child_frame_id
    geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
        float64 x
        float64 y
        float64 z
    geometry_msgs/Quaternion rotation
        float64 x
        float64 y
        float64 z
        float64 w
*/

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "robot_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "robot_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
    loop_rate.sleep();
 
  }
}
