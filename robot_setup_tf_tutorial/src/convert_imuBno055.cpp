#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sti_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <sti_msgs/Velocities.h>

sensor_msgs::Imu raw_bno , raw_0; 
bool have_raw_vel, have_cmd_vel ;
bool raw_vel_0 = true ;
bool enable_pub = false ;
int dem = 0 ;

void getRaw(const sti_msgs::Imu& raw)
{   
    raw_bno.linear_acceleration = raw.linear_acceleration ;
    raw_bno.angular_velocity = raw.angular_velocity ;
    enable_pub = true;
}


void call2(const sti_msgs::Velocities raw){ // 35hx
   
    if ( raw.linear_x == 0 && \
         raw.linear_y == 0 && \
         raw.angular_z == 0    )
    {   
        dem ++; 
        if( dem > 175 ) raw_vel_0 = true ; // wait 5s : 5*35 = 75
        // ROS_INFO("dem = %d",dem);
    }
    else {
        raw_vel_0 = false ;
        dem = 0 ;
    }

    have_raw_vel = true ;
}

// void call3(const sti_msgs/Velocities vel){

// }


int main(int argc, char** argv){
    ros::init(argc, argv, "convert_imuBNO");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("raw_imu_bno055", 100, getRaw); // 62hz
    ros::Subscriber sub2 = n.subscribe("raw_vel", 100, call2); 
    // ros::Subscriber sub3 = n.subscribe("cmd_vel", 100, call3); 
    // ros::Subscriber sub4 = n.subscribe("cmd_vel_ps2", 100, call4);
    ros::Publisher pub_raw = n.advertise<sensor_msgs::Imu>("/raw_imu_BNO", 60);
    ros::Rate r(35.0);

    while(n.ok()){
        if (enable_pub == true){
            if(have_raw_vel == true && raw_vel_0 == false ){
                ROS_DEBUG("ekf imu");
                raw_bno.header.stamp = ros::Time::now();
                raw_bno.header.frame_id = "imu_bno" ;

                pub_raw.publish(raw_bno); 
                enable_pub = false ;
                have_raw_vel = false ;
            }
            else{
                ROS_DEBUG("NO Ekf imu");
                raw_0.header.stamp = ros::Time::now();
                raw_0.header.frame_id = "imu_bno" ;
                // chong xoay map khi khoi dong
                raw_0.orientation.w = 1 ;
                raw_0.linear_acceleration.x = 0.000001 ;
                raw_0.linear_acceleration.y = 0.000001 ;
                raw_0.linear_acceleration.z = 92.000000 ;
                raw_0.angular_velocity.x = 0.000001 ;
                raw_0.angular_velocity.y = 0.000001 ;
                raw_0.angular_velocity.z = 0.000001 ;

                pub_raw.publish(raw_0); 
            }

        }

        ros::spinOnce();
        r.sleep();      
    }
    
}

