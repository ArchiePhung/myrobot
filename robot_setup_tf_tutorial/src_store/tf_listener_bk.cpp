/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
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

/*  msg : geometry_msgs/QuaternionStamped

    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Quaternion quaternion
      float64 x
      float64 y
      float64 z
      float64 w
*/

nav_msgs::Odometry odom_t265 ; 
geometry_msgs::PointStamped odom_conver ; 
geometry_msgs::PoseStamped  pose_conver;
geometry_msgs::Vector3Stamped linear_vector_conver ;
geometry_msgs::Vector3Stamped angular_vector_conver ;

void getOdom_t265(const nav_msgs::Odometry& odom)
{  
  odom_t265 = odom ;
}

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_t265 frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped t265_point;  

  //we'll just use the most recent transform available for our simple example
  t265_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  t265_point.header.seq  = odom_t265.header.seq ;
  // t265_point.header.stamp  = odom_t265.header.stamp ;
  t265_point.header.frame_id = "base_footprint";

  t265_point.point.x = odom_t265.pose.pose.position.x;
  t265_point.point.y = odom_t265.pose.pose.position.y;
  t265_point.point.z = odom_t265.pose.pose.position.z;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("tam_robot", t265_point, base_point);

    ROS_INFO("base_footprint: (%.2f, %.2f. %.2f) --> tam_robot: (%.2f, %.2f, %.2f) at time %.2f", 
        t265_point.point.x, t265_point.point.y, t265_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());

    odom_conver = base_point ;
    odom_conver.header.frame_id = "odom";

  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"t265_link\" to \"base_link\": %s", ex.what());
  }
}

void transformPose(const tf::TransformListener& listener){
  geometry_msgs::PoseStamped t265_pose;   
 
  // t265_pose.header.stamp = ros::Time();
  t265_pose.header.stamp  = odom_t265.header.stamp ;

  t265_pose.header.seq  = odom_t265.header.seq ;
  t265_pose.header.frame_id  = "t265_link";

  t265_pose.pose.position    = odom_t265.pose.pose.position;
  t265_pose.pose.orientation = odom_t265.pose.pose.orientation ;

  try{
    geometry_msgs::PoseStamped base_pose;
    listener.transformPose("base_link", t265_pose, base_pose);

    ROS_INFO("t265 position: (%.2f, %.2f. %.2f) --> base position: (%.2f, %.2f, %.2f)\n", 
      t265_pose.pose.position.x, t265_pose.pose.position.y, t265_pose.pose.position.z,
      base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z);
    
    ROS_INFO("t265 orientation: (%.2f, %.2f. %.2f. %.2f) --> base orientation: (%.2f, %.2f, %.2f. %.2f)\n", 
      t265_pose.pose.orientation.x, t265_pose.pose.orientation.y, t265_pose.pose.orientation.z,t265_pose.pose.orientation.w,
      base_pose.pose.orientation.x, base_pose.pose.orientation.y, base_pose.pose.orientation.z,base_pose.pose.orientation.w);

    pose_conver = base_pose ;
    odom_conver.header.frame_id = "odom";

  }
  catch(tf::TransformException& ex){
    ROS_ERROR("ERROR transformPose: %s", ex.what());
  }
}

void transformVector_linear(const tf::TransformListener& listener){
  geometry_msgs::Vector3Stamped t265_linear_vector;  

  // t265_linear_vector.header.stamp = ros::Time();
  t265_linear_vector.header.stamp  = odom_t265.header.stamp ;
  t265_linear_vector.header.seq  = odom_t265.header.seq ;
  t265_linear_vector.header.frame_id  = "t265_link";

  t265_linear_vector.vector  = odom_t265.twist.twist.linear;

  try{
    geometry_msgs::Vector3Stamped base_linear_vector;
    listener.transformVector("base_link", t265_linear_vector, base_linear_vector);

    ROS_INFO("t265 linear: (%.2f, %.2f. %.2f) --> base linear: (%.2f, %.2f, %.2f)\n", 
      t265_linear_vector.vector.x, t265_linear_vector.vector.y, t265_linear_vector.vector.z,
      base_linear_vector.vector.x, base_linear_vector.vector.y, base_linear_vector.vector.z);

    linear_vector_conver  = base_linear_vector ;
    linear_vector_conver.header.frame_id = "odom";

  }
  catch(tf::TransformException& ex){
    ROS_ERROR("ERROR transformVector_linear: %s", ex.what());
  }
}

void transformVector_angular(const tf::TransformListener& listener){
  geometry_msgs::Vector3Stamped t265_angular_vector;  

  // t265_angular_vector.header.stamp = ros::Time();
  t265_angular_vector.header.stamp  = odom_t265.header.stamp ;
  t265_angular_vector.header.seq  = odom_t265.header.seq ; 
  t265_angular_vector.header.frame_id  = "t265_link";
  t265_angular_vector.vector  = odom_t265.twist.twist.angular;

  try{
    geometry_msgs::Vector3Stamped base_angular_vector;
    listener.transformVector("base_link", t265_angular_vector, base_angular_vector);

    ROS_INFO("t265 angular: (%.2f, %.2f. %.2f) --> base angular: (%.2f, %.2f, %.2f)\n", 
      t265_angular_vector.vector.x, t265_angular_vector.vector.y, t265_angular_vector.vector.z,
      base_angular_vector.vector.x, base_angular_vector.vector.y, base_angular_vector.vector.z);
    
    angular_vector_conver  = base_angular_vector ;
    angular_vector_conver.header.frame_id = "odom";

  }
  catch(tf::TransformException& ex){
    ROS_ERROR("ERROR transformVector_angular: %s", ex.what());
  }
}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "robot_tf_listener");


  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/t265/odom/sample", 100, getOdom_t265);
  // ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry >("odomT265_conver", 100);
  ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry >("odomT265_conver", 100);
  ros::Rate loop_rate(50);

  nav_msgs::Odometry  vitri ;

  tf::TransformListener listener(ros::Duration(10));
  ros::Timer timer = n.createTimer(ros::Duration(0.01), boost::bind(&transformPoint, boost::ref(listener)));
  // ros::Timer timer1 = n.createTimer(ros::Duration(0.01), boost::bind(&transformPose, boost::ref(listener)));
  // ros::Timer timer2 = n.createTimer(ros::Duration(0.01), boost::bind(&transformVector_linear, boost::ref(listener)));
  // ros::Timer timer3 = n.createTimer(ros::Duration(0.01), boost::bind(&transformVector_angular, boost::ref(listener)));

  while (ros::ok()){

    // point
    vitri.header = odom_conver.header ;
    vitri.child_frame_id = "tam_robot" ;
    vitri.pose.pose.position = odom_conver.point ;
    vitri.pose.pose.orientation = odom_t265.pose.pose.orientation ;
    vitri.pose.covariance = odom_t265.pose.covariance ;
    vitri.twist  = odom_t265.twist;

    //pose
    // vitri.header = pose_conver.header ;
    // vitri.child_frame_id = "base_footprint" ;

    // vitri.pose.pose.position = pose_conver.pose.position  ;
    // vitri.pose.pose.orientation = pose_conver.pose.orientation ;
    // vitri.pose.covariance = odom_t265.pose.covariance ;

    // vitri.twist.twist.linear  = linear_vector_conver.vector;
    // vitri.twist.twist.angular  = angular_vector_conver.vector;
    // vitri.twist.covariance =  odom_t265.twist.covariance ;
  

    chatter_pub.publish(vitri);

    loop_rate.sleep();
    ros::spinOnce();
    
  }
}
