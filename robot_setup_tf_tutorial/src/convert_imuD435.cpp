#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

sensor_msgs::Imu acc_d435 , gyro_d435 ,raw_imu_d435;

void getAcc_d435(const sensor_msgs::Imu& acc)
{   
    acc_d435.linear_acceleration = acc.linear_acceleration ; 
    // ROS_INFO("acc...");
}

void getGyro_d435(const sensor_msgs::Imu& gyro)
{   
    gyro_d435.angular_velocity = gyro.angular_velocity ;
    // ROS_INFO("gyro...");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "convert_imuD435");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/d435/accel/sample", 100, getAcc_d435); // 62hz
    ros::Subscriber sub2 = n.subscribe("/d435/gyro/sample", 1000, getGyro_d435); // 200hz
    ros::Publisher pub_raw = n.advertise<sensor_msgs::Imu>("/raw_imu_d435", 60);
    ros::Rate r(60.0);

    while(n.ok()){
        // ROS_INFO("converting...");
        raw_imu_d435.header.stamp = ros::Time::now();
        raw_imu_d435.header.frame_id = "imu_d435" ;
        raw_imu_d435.angular_velocity = gyro_d435.angular_velocity;
        raw_imu_d435.linear_acceleration =acc_d435.linear_acceleration  ;
        //publish the message
        pub_raw.publish(raw_imu_d435); 

        ros::spinOnce();
        r.sleep();      
    }
}

