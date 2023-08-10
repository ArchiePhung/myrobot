#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <time.h>

double dt_ht, dt_tr , deta_dt ; 
double dt_ht1, dt_tr1 , deta_dt1 ; 

nav_msgs::Odometry odom_t265;
sensor_msgs::Imu gyro_t265 ;
geometry_msgs::Point t265_msg;

void getOdom_t265(const nav_msgs::Odometry& odom)
{ 
    odom_t265 = odom ;   
    dt_ht++ ;
}

void getGyro_t265(const sensor_msgs::Imu& gyro)
{   
    dt_ht1++;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "checkErrorT265");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/t265/odom/sample", 200, getOdom_t265);   //200hz
    ros::Subscriber sub1 = n.subscribe("/t265/gyro/sample", 200, getGyro_t265); // 200hz
    ros::Publisher info_t265 = n.advertise<geometry_msgs::Point>("/info_t265", 1000);
    ros::Rate r(1.0);

    while(n.ok()){
       
        deta_dt = dt_ht - dt_tr ; //hz
        dt_tr = dt_ht ;

        deta_dt1 = dt_ht1 - dt_tr1 ; //hz
        dt_tr1 = dt_ht1 ;

        // ROS_INFO("hz_odom = %f , hx_gyro = %f \n", deta_dt,deta_dt1 );

        time_t t = time(NULL);
        struct tm tm = *localtime(&t);
        // ROS_INFO("now: %d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

        if(deta_dt == 0 && deta_dt1 == 0 ){
            ROS_INFO(" T265 chua CHAY !!!");
            t265_msg.x = 1; 
        }

        else if((deta_dt > 100) && (deta_dt1 > 100) && (!isnan(odom_t265.pose.pose.position.x)) ){
            ROS_INFO(" T265 OK , checking...");
            t265_msg.x = 2; 
        }  

        else if(deta_dt == 0 && deta_dt1 > 100 && !isnan(odom_t265.pose.pose.position.x)){
            ROS_INFO(" T265 CANH BAO : ODOM=0 + TOI !!!");
            t265_msg.x = 3; 
        }
        
        else if(isnan(odom_t265.pose.pose.position.x)){
            ROS_INFO(" T265 error...");
            t265_msg.x = 4; 
        }
        else {
            ROS_INFO(" T265 Nothing (-;-) \n");
            t265_msg.x = 0; 
        }
            
        info_t265.publish(t265_msg);

        ros::spinOnce();
        r.sleep();      
    }
}
