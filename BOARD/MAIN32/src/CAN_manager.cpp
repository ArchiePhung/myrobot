/*==================================================================================================
*   Project              :  MAIN AI CONTROL
*   Doccument            :  ESP32S 
*   FileName             :  cancontrol.cpp
*   File Description     :  Dinh nghia cac ham thu vien su dung trong cancontrol.h
*
==================================================================================================*/
/*==================================================================================================
Revision History:
Modification     
    Author                  Date D/M/Y     Description of Changes
------------------------    -----------    ---------------------------------------------------------
    Do Xuan An              30/07/2020     Tao file
------------------------    -----------    ---------------------------------------------------------
==================================================================================================*/
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "CAN_manager.h"
/*==================================================================================================
*                                     FILE VERSION CHECKS
==================================================================================================*/

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/
/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/
/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
CAN_device_t CAN_cfg; // config
CAN_frame_t tx_frame; // transmit
CAN_frame_t rx_frame; // receive
byte_b meo;
CAN_frame_t compared; // receive
/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
/*==================================================================================================
*                                        LOCAL FUNCTION 
==================================================================================================*/
/*==================================================================================================
*                                      GLOBAL FUNCTIONS
==================================================================================================*/


CAN_manager::CAN_manager(CAN_speed_t baud_speed,gpio_num_t tx,gpio_num_t rx, CAN_frame_format_t frame,uint32_t can_id, uint8_t send_size){
    _baud_speed = baud_speed;
    _tx = tx;
    _rx =rx;
    _frame = frame;
    _can_id = can_id;
    _send_size = send_size;
}

void CAN_manager::CAN_prepare(void) {
    CAN_cfg.speed =_baud_speed;
    CAN_cfg.tx_pin_id = _tx;
    CAN_cfg.rx_pin_id = _rx;
    /* create a queue for CAN receiving */
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
    //initialize CAN Module
    ESP32Can.CANInit();
    // config send data
    tx_frame.FIR.B.FF = _frame;
    tx_frame.MsgID = _can_id;
    tx_frame.FIR.B.DLC = _send_size; // du lieu truyen tu 1-8
}

bool CAN_manager::CAN_ReceiveFrom(int from_ID){
    if(xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3*portTICK_PERIOD_MS) == pdTRUE){
        if(rx_frame.FIR.B.RTR != CAN_RTR){
            if(rx_frame.MsgID == from_ID){
                return true;
            }
        }
    }
    return false;
}

bool CAN_manager::CAN_Send(){
    static int time_mt = millis();
    if(compared.data.u32[0] == tx_frame.data.u32[0] && compared.data.u32[1] == tx_frame.data.u32[1] && (millis() - time_mt)<60){
        return false;
    }else 
    {
        if(ESP32Can.CANWriteFrame(&tx_frame)){}
        compared = tx_frame;
        time_mt = millis();
    }
    return true;
}

uint8_t CAN_manager::GetByteReceived(unsigned char pos){return rx_frame.data.u8[pos];}

void CAN_manager::SetByteTransmit(uint8_t dat,unsigned char pos){tx_frame.data.u8[pos] = dat;}
//------------------------------------------END FILE----------------------------------------------//
