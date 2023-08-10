
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sti_msgs/Velocities.h>


sti_msgs::Velocities vel ;
bool enable_vel ;
float vel_dt_;
float x_pos_;
float y_pos_;
float heading_;

void velCallback(const sti_msgs::Velocities& vel_)
{
    vel = vel_ ; 
    enable_vel = true ;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_encoder");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("raw_vel", 50, velCallback);
    ros::Publisher  pub = n.advertise<nav_msgs::Odometry>("raw_odom", 50);

    tf::TransformBroadcaster        odom_broadcaster;
    tf2::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans      ;
    nav_msgs::Odometry odom ;

    ros::Time current_time,last_vel_time_ ;
    ros::Rate r(40);

    while(n.ok()){
        ros::spinOnce(); 
        if(enable_vel == true){
            enable_vel = false ;
            ros::Time current_time = ros::Time::now();

            vel_dt_ = (current_time - last_vel_time_).toSec();
            last_vel_time_ = current_time;

            double delta_heading = vel.angular_z * vel_dt_; //radians
            double delta_x = (vel.linear_x * cos(heading_) - vel.linear_y * sin(heading_)) * vel_dt_; //m
            double delta_y = (vel.linear_x * sin(heading_) + vel.linear_y * cos(heading_)) * vel_dt_; //m

            //calculate current position of the robot
            x_pos_ += delta_x;
            y_pos_ += delta_y;
            heading_ += delta_heading;

            //calculate robot's heading in quaternion angle
            //ROS has a function to calculate yaw in quaternion angle
            odom_quat.setRPY(0,0,heading_);

            // odom_trans.header.frame_id = "odom";
            // odom_trans.child_frame_id = "base_footprint";
            // odom_trans.header.stamp = current_time;
            // odom_trans.transform.translation.x = x_pos_;
            // odom_trans.transform.translation.y = y_pos_;
            // odom_trans.transform.rotation.z = odom_quat.z();
            // odom_trans.transform.rotation.w = odom_quat.w();
            // odom_broadcaster.sendTransform(odom_trans);

            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_footprint";
            odom.pose.pose.position.x = x_pos_;
            odom.pose.pose.position.y = y_pos_;
            odom.pose.pose.orientation.z = odom_quat.z();
            odom.pose.pose.orientation.w = odom_quat.w();
            odom.pose.covariance[0] = 0.001;
            odom.pose.covariance[7] = 0.001;
            odom.pose.covariance[35] = 0.001;

            //linear speed from encoders
            odom.twist.twist.linear.x = vel.linear_x;
            odom.twist.twist.linear.y = vel.linear_y;
            //angular speed from encoders
            odom.twist.twist.angular.z = vel.angular_z;
            odom.twist.covariance[0] = 0.0001;
            odom.twist.covariance[7] = 0.0001;
            odom.twist.covariance[35] = 0.0001;

            pub.publish(odom);
        }
        r.sleep();
        
    }
}

