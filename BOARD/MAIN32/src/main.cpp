#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "CAN_manager.h"
#include "MAIN_controller.h"
#include "WiFi.h"

#include "ros.h"
#include "ros/time.h"
#include "message_pkg/App_request.h"
#include "geometry_msgs/Twist.h"

#define FREQUENCY_PUB_POWER_INFO 4
#define FREQUENCY_PUB_OC_INFO    4
#define FREQUENCY_PUB_HC_INFO    24

void App_request_Callback(const message_pkg::App_request &data);
void check_spinOnce();

ros::NodeHandle nodeHandle;
message_pkg::App_request App_request;
geometry_msgs::Twist robot_vel;

uint8_t mode;
uint8_t action;
float vel;

ros::Subscriber<message_pkg::App_request> App_request_sub("App_request", App_request_Callback);
// TaskHandle_t Task0;

MAIN_controller* Main_Ctrl = new MAIN_controller();

// void Task0_code( void * pvParameters ){
//   for(;;){
//     OC_Ctrl->OC_CAN_Receive();
//     vTaskDelay(2);
//   }
// }

// void create_task() {
//   xTaskCreatePinnedToCore(
//                     Task0_code,   /* Task function. */
//                     "Task0",     /* name of task. */
//                     10000,       /* Stack size of task */
//                     NULL,        /* parameter of the task */
//                     1,           /* priority of the task */
//                     &Task0,      /* Task handle to keep track of created task */
//                     0);          /* pin task to core 1 */
// }

void App_request_Callback(const message_pkg::App_request &data)
{
    // - Control main
    mode = data.mode;
    action = data.action;
    if(action == 1 && action == 3){                      // forward and backward
      robot_vel.linear.x = data.vel;
      robot_vel.angular.z = 0;
    }
    else if(action == 2 && action == 4){
      robot_vel.linear.x = 0;
      robot_vel.angular.z = data.vel;
    }
    else{
      robot_vel.linear.x = 0;
      robot_vel.angular.z = 0;
    }

}

void MAIN_controller::robot_run(){
  Main_Ctrl->action = action;
  if(Main_Ctrl->action == 1) robot_forward();
  else if(Main_Ctrl->action == 3) robot_backward();
  else robot_stop();
}

void setup() {
  WiFi.mode(WIFI_OFF);
  delay(50);
  Main_Ctrl->setupBegin();
  btStop();
  delay(50);
  nodeHandle.initNode();
  nodeHandle.getHardware()->setBaud(57600);

  nodeHandle.subscribe(App_request_sub);

  // create_task();
  // delay(100);
}

void loop() {
  Main_Ctrl->robot_run();
  
  nodeHandle.spinOnce();
  
  // OC_Ctrl->tryRun();
  // OC_Ctrl->debug();
}