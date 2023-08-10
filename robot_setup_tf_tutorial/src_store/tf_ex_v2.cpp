#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
// #include <math.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <time.h>
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

nav_msgs::Odometry odom_t265 ;
tf2::Quaternion q_orig ;
geometry_msgs::PoseStamped _pose_robot;
geometry_msgs::PoseStamped _pose_haha;

void getOdom_t265(const nav_msgs::Odometry& odom)
{  
  odom_t265 = odom ;
}

#define pi 3.141592653589793238462643383279502884

void transformPose_t265_to_robot(const tf::TransformListener& listener){
  geometry_msgs::PoseStamped t265_pose;   
 
  t265_pose.header.stamp = ros::Time();
  t265_pose.header.frame_id  = "t265_link";

  /*
      t265_pose.pose.position.x=-0.08755 ;
      t265_pose.pose.position.y=-0.0184349;
      t265_pose.pose.position.z=-0.003777;
      t265_pose.pose.orientation.x=0.63418 ;
      t265_pose.pose.orientation.y=-0.308281;
      t265_pose.pose.orientation.z=0.63336;
      t265_pose.pose.orientation.w=0.31879;
  */
    t265_pose.pose.position = odom_t265.pose.pose.position ;
    // t265_pose.pose.position.x = odom_t265.pose.pose.position.z ;
    // t265_pose.pose.position.y = odom_t265.pose.pose.position.y ;
    // t265_pose.pose.position.z = odom_t265.pose.pose.position.x ;

    t265_pose.pose.orientation = odom_t265.pose.pose.orientation ;

  try{
    
    listener.transformPose("robot_link", t265_pose, _pose_robot);

    ROS_INFO("t265 position (%f, %f. %f) --> robot position: (%f, %f, %f)\n", 
      t265_pose.pose.position.x, t265_pose.pose.position.y, t265_pose.pose.position.z,
      _pose_robot.pose.position.x, _pose_robot.pose.position.y, _pose_robot.pose.position.z);
    
    ROS_INFO("t265 orientation: (%f, %f. %f. %f) --> robot orientation: (%f, %f. %f. %f))\n", 
      t265_pose.pose.orientation.x, t265_pose.pose.orientation.y, t265_pose.pose.orientation.z,t265_pose.pose.orientation.w,
      _pose_robot.pose.orientation.x, _pose_robot.pose.orientation.y, _pose_robot.pose.orientation.z,_pose_robot.pose.orientation.w);

  }
  catch(tf::TransformException& ex){
    ROS_ERROR("ERROR transformPose: %s", ex.what());
  }
}

void transformPose_robot_to_haha(const tf::TransformListener& listener){
  geometry_msgs::PoseStamped haha_pose;   
 
  haha_pose.header.stamp = ros::Time();
  haha_pose.header.frame_id  = "robot_link";
  haha_pose.pose.position = _pose_robot.pose.position ;
  haha_pose.pose.orientation = _pose_robot.pose.orientation ;

  try{
    geometry_msgs::PoseStamped _pose_robot;
    listener.transformPose("haha_link", haha_pose, _pose_haha);

    ROS_INFO("robot position (%f, %f. %f) --> haha position: (%f, %f, %f)\n", 
      haha_pose.pose.position.x, haha_pose.pose.position.y, haha_pose.pose.position.z,
      _pose_haha.pose.position.x, _pose_haha.pose.position.y, _pose_haha.pose.position.z);
    
    ROS_INFO("robot orientation: (%f, %f. %f. %f) --> haha orientation: (%f, %f. %f. %f))\n", 
      haha_pose.pose.orientation.x, haha_pose.pose.orientation.y, haha_pose.pose.orientation.z,haha_pose.pose.orientation.w,
      _pose_haha.pose.orientation.x, _pose_haha.pose.orientation.y, _pose_haha.pose.orientation.z,_pose_haha.pose.orientation.w);

  }
  catch(tf::TransformException& ex){
    ROS_ERROR("ERROR transformPose: %s", ex.what());
  }
}

float px_t265,py_t265,oz_t265 ; 
float px_robot,py_robot,goc_robot ; 
double roll,roll_ht,roll_set, pitch, yaw, yaw_ht,yaw_set, yaw_rad;
// tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);

void diy_tranform(){

  tf::Quaternion q1(odom_t265.pose.pose.orientation.x,odom_t265.pose.pose.orientation.y,odom_t265.pose.pose.orientation.z,odom_t265.pose.pose.orientation.w);
  tf::Matrix3x3 m(q1);
  m.getRPY(roll, pitch, yaw_ht); // rad
  
 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_listener");
    ros::NodeHandle n ;
    ros::Subscriber sub = n.subscribe("/t265/odom/sample", 100, getOdom_t265);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_robot", 50);
    nav_msgs::Odometry odom;
    tf2::Quaternion odom_quat;

    tf::TransformBroadcaster odom_broadcaster;
    ros::Rate loop_rate(10);


  // tf::TransformListener listener(ros::Duration(10));
  // ros::Timer timer1 = n.createTimer(ros::Duration(1.0), boost::bind(&transformPose_t265_to_robot, boost::ref(listener)));
  // ros::Timer timer2 = n.createTimer(ros::Duration(1.0), boost::bind(&transformPose_robot_to_haha, boost::ref(listener)));

    // tf2_ros::TransformBroadcaster tfb;
    // geometry_msgs::TransformStamped transformStamped;

    // transformStamped.header.frame_id = "t265_link";
    // transformStamped.child_frame_id = "robot_link";
    // tf2::Quaternion q;

    // double goc = 0 ;
    double d = 0.205 ; // met 

    // transformStamped.transform.translation.x = 0.0;
    // transformStamped.transform.translation.y = d*sin(goc);
    // transformStamped.transform.translation.z = -d*cos(goc);
    // q.setRPY(0, 0, 0);
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();
    
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q;


    ros::Time t1,t2 ;
    int buoc=0 ;
    t1=ros::Time::now();

    ros::Time current_time, last_time, last_vel_time_;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    double x_ht = -0.205 ,  x_tr = -0.205 ; //px_robot  ;
    double y_ht = 0,        y_tr = 0 ;            //py_robot  ;
    double goc_ht = 0 ,     goc_tr = 0 ;

  while (ros::ok()){
    // dinh dang frame 
    if(buoc == 0){ 
      t2=ros::Time::now();
      diy_tranform();   
      yaw_set = 0 ;
      roll_set = 0 ;
      if( t2.toSec()-t1.toSec() > 5){
        yaw_set = yaw_ht ;
        yaw_set = (180/pi)*yaw_set;
        roll_set = roll_ht ;
        buoc = 1 ;
      } 
 
    }

    if( buoc == 1){

        // chuyen doi toa do 
        diy_tranform();

        yaw     = (180/pi)*yaw ;
        // yaw_set = (180/pi)*yaw_set ;
        yaw_ht  = (180/pi)*yaw_ht ;

        if(yaw_set >=0 ){
            
            if(yaw_ht >= -180 && yaw_ht <=  yaw_set -180  ){
                 yaw =  (180 - yaw_set) + (180 + yaw_ht) ;
            }
            else{
                yaw =  yaw_ht - yaw_set;
           }
        }

        if(yaw_set <0 ){
            
            if(yaw_ht <= 180 && yaw_ht >=  yaw_set + 180  ){
                 yaw =  (180 + yaw_set) + (180 - yaw_ht) ;
            }
            else{
                yaw =  yaw_ht - yaw_set;
           }
        }
        // goc_robot = (180/pi)*yaw ;
        yaw_rad = yaw*(pi/180);
        // if(yaw  >=0 && yaw  <= 90){           
            px_robot = odom_t265.pose.pose.position.x - d*cos(yaw_rad);
            py_robot = odom_t265.pose.pose.position.y - d*sin(yaw_rad);
            ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(1)= %f yaw_rad= %f )\n ",px_robot,py_robot,yaw,yaw_rad );
        // }
        // if(yaw  >90 && yaw  <= 180){
        //     px_robot = odom_t265.pose.pose.position.x + d*cos(180 - yaw_rad);
        //     py_robot = odom_t265.pose.pose.position.y - d*sin(180 - yaw_rad);
        //     ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(2)= %f) \n ",px_robot,py_robot,yaw );
        // }
        // if(yaw  >=-180 && yaw  <= -90){
        //     px_robot = odom_t265.pose.pose.position.x + d*cos(-180 -yaw_rad);
        //     py_robot = odom_t265.pose.pose.position.y - d*sin(-180 -yaw_rad);
        //     ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(3)= %f) \n ",px_robot,py_robot,yaw );
        // }
        // if(yaw  >-90 && yaw  <0){
        //     px_robot = odom_t265.pose.pose.position.x - d*cos(yaw_rad);
        //     py_robot = odom_t265.pose.pose.position.y - d*sin(yaw_rad);
            // ROS_INFO("robot: (x_rb= %f y_rb= %f yaw_rb(4)= %f) \n ",px_robot,py_robot,yaw );
        // }

        // ROS_INFO("t265: (x= %f y= %f yaw_set= %f yaw_ht = %f) v35 \n",odom_t265.pose.pose.position.x, odom_t265.pose.pose.position.y,yaw_set,yaw_ht);
        // ROS_INFO("buoc= %d t1= %f t2= %f \n",buoc,t1.toSec(),t2.toSec());
    }
   
//--------------- publish odom robot------------------------------------// 
    ROS_INFO("-- D7 --\n");

    // current_time = ros::Time::now();
    // double dt = (current_time - last_time).toSec();
    // x_ht = px_robot ;
    // y_ht = py_robot ; 
    // goc_ht = yaw_rad ;

    // double vx = (( x_ht- x_tr )/ dt) ; // m/s 
    // double vy = (( y_ht- y_tr )/ dt) ; // m/s 
    // double v_yaw = (( goc_ht- goc_tr )/ dt) ; // rad/s 

    // ROS_INFO("vx = %f vy = %f v_yaw = %f \n",vx,vy,v_yaw);

    // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw_rad);

    // // publish the transform over tf
    // transformStamped.header.frame_id = "odom";
    // transformStamped.child_frame_id = "base_footprint";
    // transformStamped.transform.translation.x = px_robot ;
    // transformStamped.transform.translation.y = py_robot;
    // transformStamped.transform.translation.z = 0.0;
    // transformStamped.transform.rotation = odom_quat;
    // //send the transform
    // // tfb.sendTransform(transformStamped);
    

    // //publish the odometry message over ROS
    // odom.header.stamp = current_time;
    // odom.header.frame_id = "odom";
    // odom.child_frame_id = "base_footprint";

    // //set the position
    // odom.pose.pose.position.x = px_robot ;
    // odom.pose.pose.position.y = py_robot;
    // odom.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation = odom_quat ;

    // //set the velocity
    // odom.twist.twist.linear.x = vx;
    // odom.twist.twist.linear.y = vy;
    // odom.twist.twist.angular.z = v_yaw;
    // //publish the message
    // odom_pub.publish(odom);
    

    // last_time = current_time ;
    // x_tr = x_ht  ;
    // y_tr = y_ht  ;
    // goc_tr = goc_ht ;

    // loop_rate.sleep();
    // ros::spinOnce(); 

  }
}