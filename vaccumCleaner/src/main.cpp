#include <Arduino.h>
#include "ros.h"
#include <message_pkg/Vel_msg.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include "param_config.h"
#include "pin_config.h"
#include "main.h"
#include <util/atomic.h>

// variables
int w_r=0, w_l=0;
double speed_ang=0, speed_lin=0;
double wheel_rad = 0.0325, wheel_sep = 0.295;

long tim = 0;

message_pkg::Vel_msg vel_info;

ESP32_Controller ESP32_Main;
Robot vaccumCleanerRobot;

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

void messageCb(const geometry_msgs::Twist& msg){
   speed_ang = msg.angular.z;
   speed_lin = msg.linear.x;
   w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
   w_r = constrain(w_r, -255, 255);
   vel_info.pwm_right_motor.data = w_r;
   
   w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
   // w_l = (w_l >= 255) ? 255 : w_l;
   w_l = constrain(w_l, -255, 255);
   vel_info.pwm_left_motor.data = w_l;
} 

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);
ros::Publisher pwm_info("/pwm_info", &vel_info);

void ESP32_Controller::pin_init()
{
   pinMode(LEFT_MOTOR_PIN_H, OUTPUT);
   pinMode(LEFT_MOTOR_PIN_L, OUTPUT);
   pinMode(LEFT_ENABLE_MOTOR, OUTPUT);
   pinMode(RIGHT_MOTOR_PIN_H, OUTPUT);
   pinMode(RIGHT_MOTOR_PIN_L, OUTPUT);
   pinMode(RIGHT_ENABLE_MOTOR, OUTPUT);

   pinMode(LEFT_ENCA, INPUT);
   pinMode(LEFT_ENCB, INPUT);
   pinMode(RIGHT_ENCA, INPUT);
   pinMode(RIGHT_ENCA, INPUT);
}

void Robot::stop(){
   digitalWrite(LEFT_MOTOR_PIN_H, 0);
   digitalWrite(LEFT_MOTOR_PIN_L, 0);
   analogWrite(LEFT_ENABLE_MOTOR, 0);
   digitalWrite(RIGHT_MOTOR_PIN_H, 0);
   digitalWrite(RIGHT_MOTOR_PIN_L, 0);
   analogWrite(RIGHT_ENABLE_MOTOR, 0);
}

void Robot::run(int Motor_speeda, int Motor_speedb)
{
    if( Motor_speeda >= 0 && Motor_speedb >= 0){
      // ledcWrite(0,Motor_speeda);
      // ledcWrite(1,0);
      // ledcWrite(2,Motor_speedb);
      // ledcWrite(3,0);
      digitalWrite(LEFT_MOTOR_PIN_H, 1);
      digitalWrite(LEFT_MOTOR_PIN_L, 0);
      analogWrite(LEFT_ENABLE_MOTOR, Motor_speeda);
      digitalWrite(RIGHT_MOTOR_PIN_H, 1);
      digitalWrite(RIGHT_MOTOR_PIN_L, 0);
      analogWrite(RIGHT_ENABLE_MOTOR, Motor_speedb);
    }

    else if(Motor_speeda >= 0 && Motor_speedb < 0){
      digitalWrite(LEFT_MOTOR_PIN_H, 1);
      digitalWrite(LEFT_MOTOR_PIN_L, 0);
      analogWrite(LEFT_ENABLE_MOTOR, Motor_speeda);
      digitalWrite(RIGHT_MOTOR_PIN_H, 0);
      digitalWrite(RIGHT_MOTOR_PIN_L, 1);
      analogWrite(RIGHT_ENABLE_MOTOR, abs(Motor_speedb));   
    }

    else if( Motor_speeda < 0 && Motor_speedb >= 0){
      digitalWrite(LEFT_MOTOR_PIN_H, 0);
      digitalWrite(LEFT_MOTOR_PIN_L, 1);
      analogWrite(LEFT_ENABLE_MOTOR, abs(Motor_speeda));
      digitalWrite(RIGHT_MOTOR_PIN_H, 1);
      digitalWrite(RIGHT_MOTOR_PIN_L, 0);
      analogWrite(RIGHT_ENABLE_MOTOR, Motor_speedb);
    }

    else{
      digitalWrite(LEFT_MOTOR_PIN_H, 0);
      digitalWrite(LEFT_MOTOR_PIN_L, 1);
      analogWrite(LEFT_ENABLE_MOTOR, abs(Motor_speeda));
      digitalWrite(RIGHT_MOTOR_PIN_H, 0);
      digitalWrite(RIGHT_MOTOR_PIN_L, 1);
      analogWrite(RIGHT_ENABLE_MOTOR, abs(Motor_speedb));   
    }
}

void ESP32_Controller::launch_init()
{
   // ledcSetup(0,1024,10);
   // ledcSetup(1,1024,10);
   // ledcSetup(2,1024,10);
   // ledcSetup(3,1024,10);

   // ledcAttachPin(LEFT_MOTOR_PIN_H,0);
   // ledcAttachPin(LEFT_MOTOR_PIN_L,1);
   // ledcAttachPin(RIGHT_MOTOR_PIN_H,2);
   // ledcAttachPin(RIGHT_MOTOR_PIN_L,3);
   rb_ctrl->stop();
}

void setup()
{  
   ESP32_Main.pin_init();

   nh.initNode();
   nh.getHardware()->setBaud(57600);
   nh.subscribe(sub);
   nh.advertise(pwm_info);

   attachInterrupt(digitalPinToInterrupt(LEFT_ENCA ,readEncoder_left, RISING));

   tim = millis();
}
 
void loop()
{   
   vaccumCleanerRobot.run(w_l,w_r);

   if(millis() - tim > 1000){
      pwm_info.publish(&vel_info);
      tim = millis();
   }
   nh.spinOnce();
}

void readEncoder_left(){
   
}
