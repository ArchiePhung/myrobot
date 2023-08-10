#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tf2_listener(tfBuffer);
geometry_msgs::TransformStamped base_link_to_leap_motion; // My frames are named "base_link" and "leap_motion"

base_link_to_leap_motion = tfBuffer.lookupTransform("leap_motion", "base_link", ros::Time(0), ros::Duration(1.0) );

tf2::doTransform(robotPose, robotPose, base_link_to_leap_motion); // robotPose is the PoseStamped I want to transform