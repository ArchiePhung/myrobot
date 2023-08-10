/**
 * File : hardwareconfig.h
 * Version : 8.2.0
 * Date    : 17/06/2020
 * Author  : AnDX
 * Description :
 * 
 *******************************************************/

#ifndef __HARDWARECONFIG_H
#define __HARDWARECONFIG_H

#include <Arduino.h>
#include "HardwareSerial.h"
#include "CAN_config.h"

#define DEBUG_SERIAL_ENABLE

#ifdef DEBUG_SERIAL_ENABLE
    #define DebugSrial Serial 
#endif

#define ID_OC   0x04
#define ID_RTC_ORIGIN  0x01

#define CAN_BAUD_SPEED  CAN_SPEED_125KBPS
#define CAN_TX          GPIO_NUM_5
#define CAN_RX          GPIO_NUM_4
#define CAN_FRAME       CAN_frame_std
#define CAN_ID          ID_OC
#define CAN_SEND_SIZE   8
#define CAN_TIMER       60
#define TIME_OUT        50000

// ---------------------
#define LIMIT_SWITCH  32 //1

#define revert1 1
#define revert2 0

// -- LEFT MOTOR
// #define CTR_ENABLE1    14 //1

#if revert1
    #define CTR_PWM_H1     19 // -
    #define CTR_PWM_L1     21 // -
#else
    #define CTR_PWM_H1     21 // - origin
    #define CTR_PWM_L1     19 // -
#endif

// -- ENCODER LEFT MOTOR
#define SS_LEFT_MOTOR_A    34
#define SS_LEFT_MOTOR_B    35


// -- RIGHT MOTOR
// #define CTR_ENABLE2     25 //1

#if revert2
    #define CTR_PWM_H2     22 // -
    #define CTR_PWM_L2     23 // -
#else
    #define CTR_PWM_H2     23 // - origin
    #define CTR_PWM_L2     22 // -
#endif


// -- ENCODER RIGHT MOTOR 
#define SS_RIGHT_MOTOR_A    36
#define SS_RIGHT_MOTOR_B    39

// -- STEP MOTOR
#define STEP_ENABLE 16
#define STEP_DIR 17
#define STEP_PUL 18

#endif