#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

sensor_msgs::Imu acc_t265 , gyro_t265 ,raw_imu_t265;

void getAcc_t265(const sensor_msgs::Imu& acc)
{   
    acc_t265.linear_acceleration = acc.linear_acceleration ; 
    // ROS_INFO("acc...");
}

void getGyro_t265(const sensor_msgs::Imu& gyro)
{   
    gyro_t265.angular_velocity = gyro.angular_velocity ;
    // ROS_INFO("gyro...");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "convert_imuT265");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/t265/accel/sample", 100, getAcc_t265); // 62hz
    ros::Subscriber sub2 = n.subscribe("/t265/gyro/sample", 1000, getGyro_t265); // 200hz
    ros::Publisher pub_raw = n.advertise<sensor_msgs::Imu>("/raw_imu_t265", 60);
    ros::Rate r(60.0);


    while(n.ok()){
        // ROS_INFO("converting...");
        raw_imu_t265.header.stamp = ros::Time::now();
        raw_imu_t265.header.frame_id = "imu_t265" ;
        raw_imu_t265.angular_velocity = gyro_t265.angular_velocity;
        raw_imu_t265.linear_acceleration =acc_t265.linear_acceleration  ;
        //publish the message
        pub_raw.publish(raw_imu_t265); 

        ros::spinOnce();
        r.sleep();      
    }
    
}

