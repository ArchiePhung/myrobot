#include "MAIN_controller.h"

MAIN_controller::MAIN_controller()
{
    
}

MAIN_controller::~MAIN_controller()
{
  delete CAN_sendData;
  delete CAN_receivedData;
  delete CAN_command;
}

void MAIN_controller::setupBegin() // - OK
{
  Serial.begin(57600);
  // -
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  // - left motor
  // pinMode(CTR_ENABLE1, OUTPUT);
  pinMode(CTR_PWM_H1,  OUTPUT);
  pinMode(CTR_PWM_L1,  OUTPUT);
  // - right_motor
  // pinMode(CTR_ENABLE2, OUTPUT);
  pinMode(CTR_PWM_H2,  OUTPUT);
  pinMode(CTR_PWM_L2,  OUTPUT);

  // -------------------------------
  // -
  // digitalWrite(CTR_ENABLE1, LOW);
  digitalWrite(CTR_PWM_H1,  LOW);
  digitalWrite(CTR_PWM_L1,  LOW);
  // -
  // digitalWrite(CTR_ENABLE2, LOW);
  digitalWrite(CTR_PWM_H2,  LOW);
  digitalWrite(CTR_PWM_L2,  LOW);

  pinMode(SS_LEFT_MOTOR_A, INPUT);
  pinMode(SS_LEFT_MOTOR_B, INPUT);
  pinMode(SS_RIGHT_MOTOR_A, INPUT);
  pinMode(SS_RIGHT_MOTOR_B, INPUT);

  pinMode(STEP_ENABLE,  OUTPUT);
  pinMode(STEP_DIR,  OUTPUT);
  pinMode(STEP_PUL,  OUTPUT);

  digitalWrite(STEP_ENABLE,  HIGH);
  digitalWrite(STEP_DIR,  LOW);
  digitalWrite(STEP_PUL,  LOW);

  // ----------- PWM SETUP --------------------
  // - ledcSetup(ledChannel, freq, resolution);
  ledcSetup(channelPWM_H1, 1000, 10);
  ledcSetup(channelPWM_L1, 1000, 10);
  ledcSetup(channelPWM_H2, 1000, 10);
  ledcSetup(channelPWM_L2, 1000, 10);   

  // - ledcAttachPin(ledPin, ledChannel);
  ledcAttachPin(CTR_PWM_H1, channelPWM_H1);
  ledcAttachPin(CTR_PWM_L1, channelPWM_L1);
  ledcAttachPin(CTR_PWM_H2, channelPWM_H2);
  ledcAttachPin(CTR_PWM_L2, channelPWM_L2);
  
}

void MAIN_controller::robot_forward(){
  ledcWrite(channelPWM_H1, 1000);
  ledcWrite(channelPWM_L1, 0);
  ledcWrite(channelPWM_H2, 1000);
  ledcWrite(channelPWM_L2, 0);
}

void MAIN_controller::robot_backward(){
  ledcWrite(channelPWM_H1, 0);
  ledcWrite(channelPWM_L1, 1000);
  ledcWrite(channelPWM_H2, 0);
  ledcWrite(channelPWM_L2, 1000);
}

void MAIN_controller::robot_stop(){
  ledcWrite(channelPWM_H1, 0);
  ledcWrite(channelPWM_L1, 0);
  ledcWrite(channelPWM_H2, 0);
  ledcWrite(channelPWM_L2, 0);
}

void MAIN_controller::loop(){
  robot_forward();
  delay(2000);
  robot_stop();
  delay(1000);
  robot_backward();
  delay(2000);
  robot_stop();
  delay(1000);
}

void MAIN_controller::debug(){

}