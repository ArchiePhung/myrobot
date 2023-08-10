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
float px_robot,py_robot,oz_robot ; 
double roll,roll_ht,roll_set, pitch, yaw, yaw_ht, yaw_set;
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
  // ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  tf::TransformBroadcaster odom_broadcaster;
  ros::Rate loop_rate(10);

  
  // ros::Time current_time, last_time;
  // current_time = ros::Time::now();
  // last_time = ros::Time::now();

  // tf::TransformListener listener(ros::Duration(10));
  // ros::Timer timer1 = n.createTimer(ros::Duration(1.0), boost::bind(&transformPose_t265_to_robot, boost::ref(listener)));
  // ros::Timer timer2 = n.createTimer(ros::Duration(1.0), boost::bind(&transformPose_robot_to_haha, boost::ref(listener)));

  // tf2_ros::TransformBroadcaster tfb;
  // geometry_msgs::TransformStamped transformStamped;
  
  // transformStamped.header.frame_id = "t265_link";
  // transformStamped.child_frame_id = "robot_link";
  // tf2::Quaternion q;

  tf2_ros::TransformBroadcaster tfb2;
  geometry_msgs::TransformStamped transformStamped2;
  
  transformStamped2.header.frame_id = "t265_link";
  transformStamped2.child_frame_id = "robot2";
  tf2::Quaternion q2;


  ros::Time t1,t2 ;
  int buoc=0 ;
  t1=ros::Time::now();

  while (ros::ok()){

    if(buoc == 0){ 
      // t2=ros::Time::now();
      // diy_tranform();   
      // yaw_set = 0 ;
      // roll_set = 0 ;
      // if( t2.toSec()-t1.toSec() > 5){
      //   yaw_set = yaw_ht ;
      //   roll_set = roll_ht ;
      //   buoc = 1 ;
      // } 
       buoc = 1;
    }

    if( buoc == 1){
      diy_tranform();

      // yaw = yaw_ht - yaw_set; 
      // roll = roll_ht - roll_set ;
      // yaw = yaw_ht ; 
      // roll = roll ;

      // px_robot = 0.205*sin(roll) ;
      // py_robot = -0.205*cos(roll) ;

      // px_robot = odom_t265.pose.pose.position.x - 0.205*cos(roll) ;
      // py_robot = odom_t265.pose.pose.position.y - 0.205*sin(roll) ;

      // transformStamped.transform.translation.x = 0.0;
      // transformStamped.transform.translation.y = py_robot;
      // transformStamped.transform.translation.z = px_robot;
      // q.setRPY(0, 0, 0);
      // transformStamped.transform.rotation.x = q.x();
      // transformStamped.transform.rotation.y = q.y();
      // transformStamped.transform.rotation.z = q.z();
      // transformStamped.transform.rotation.w = q.w();
      
      // transformStamped.header.stamp = ros::Time::now();
      // tfb.sendTransform(transformStamped);

      // robot2
      roll = 0 ;
      // roll = roll ;

      px_robot = 0.205*sin(roll) ;
      py_robot = -0.205*cos(roll) ;

      transformStamped2.transform.translation.x = 0.0;
      transformStamped2.transform.translation.y = px_robot;
      transformStamped2.transform.translation.z = py_robot;
      q2.setRPY(0, 0, 0);
      transformStamped2.transform.rotation.x = q2.x();
      transformStamped2.transform.rotation.y = q2.y();
      transformStamped2.transform.rotation.z = q2.z();
      transformStamped2.transform.rotation.w = q2.w();

      // tuong duong 
      // <node pkg="tf2_ros" type="static_transform_publisher" name="t265_link_to_robot_ao" args="0 0 -0.205 0 0 0 /t265_link /robot2"/> 

      transformStamped2.header.stamp = ros::Time::now();
      tfb2.sendTransform(transformStamped2);
    }
    // ROS_INFO("robot_link: (x= %f y= %f roll= %f)\n", px_robot, py_robot,(180/pi)*roll);
    ROS_INFO("robot2: (x= %f y= %f roll= %f) v8 \n", 0.205*sin(roll), 0.205*cos(roll),(180/pi)*roll);

    ROS_INFO("buoc= %d t1= %f t2= %f \n",buoc,t1.toSec(),t2.toSec());

    loop_rate.sleep();
    ros::spinOnce(); 

  }
}