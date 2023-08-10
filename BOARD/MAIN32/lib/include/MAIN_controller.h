#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include "CAN_manager.h"
#include "hardwareconfig.h"
#include "controll_config.h"

struct CAN_SEND{ // - OC
    uint8_t oc_upload_status = 0;
    uint8_t oc_can_writeCan  = 0;                     // - Byte_0
    uint8_t oc_sensor_s1     = 0;                     // - Byte_1
    uint8_t oc_sensor_s3     = 0;                     // - Byte_2
    uint8_t oc_sensor_s4     = 0;                     // - Byte_3
    uint8_t oc_sensor_s5     = 0;                     // - Byte_4
    uint8_t oc_sensor_s6     = 0;                     // - Byte_5
    uint8_t oc_sensor_s7     = 0;                     // - Byte_6

    CAN_SEND() {}
    ~CAN_SEND(){}
};

struct CAN_RECEIVED{ // - OC
    uint8_t oc_value_readCan  = 0;
    uint8_t oc_bt_ccw         = 0;
    uint8_t oc_bt_cw          = 0;
    uint8_t oc_bn_ccw         = 0;
    uint8_t oc_bn_cw          = 0;

    CAN_RECEIVED() {}
    ~CAN_RECEIVED(){}
};

class MAIN_controller
{
    private:
        unsigned long time_gui;
        /**
        * @brief          : luu gia tri nhan/gui cua mang giao tiep CAN
        * @details        : can_id: id can cua mach
                            cmd,cmd_status: function va trang thai (0:lo lung || 1:o tren || 2:o duoi)
                            status_r: hoan thanh lenh cmd
                            err_lx: bao loi ban nang
        * @note           : 
        */
        unsigned char   cmd_request = commands::DO_NOTHING,
                        cmd_resetError = 0,
                        cmd_status = processStatus::COMMAND_DONE,
                        pre_cmd_request = commands::DO_NOTHING,
                        err_lx = ISOK;        
        // CAN_manager* CANctr = NULL;

        // - 3/5/2023 : 2 -> 1
        int FREQUENCY_sendCAN = 2;  
        unsigned long preTime_sendCAN = 0;
        uint8_t statusRev_CAN = 0;
        unsigned long saveTime_checkCAN_Rev = 0;
      
        // ----------------------------------------------
        int channelPWM_H1 = 0;
    	int channelPWM_L1 = 4;

        int channelPWM_H2 = 8;
    	int channelPWM_L2 = 12;

        uint8_t mode;
        uint8_t action;
        float vel;

    public:
        // MAIN_controller(CAN_manager*);
        // ~MAIN_controller();
        MAIN_controller();
        ~MAIN_controller();

        CAN_SEND* CAN_sendData = new CAN_SEND();
        CAN_RECEIVED* CAN_receivedData = new CAN_RECEIVED();
        CAN_RECEIVED* CAN_command = new CAN_RECEIVED();


        void setCommand(unsigned char _cmd) {cmd_request = _cmd;}
        void setCommandStatus(unsigned char _cmd_status) {cmd_status = _cmd_status;}
        void setError(unsigned char _err_lx) {err_lx = _err_lx;}

        unsigned char getError() {return err_lx;}
        unsigned char getCommandStatus() {return cmd_status;}
        unsigned char getCommand() {return cmd_request;}
        bool getSensor(int _pin) {return digitalRead(_pin);}
        
        bool the_three_timer(unsigned int timer_num, unsigned int set_value){
            static unsigned int timer_value[3]={0,0,0};
            int t = millis();
            if (t- timer_value[timer_num] >= set_value){
                timer_value[timer_num] = t;
                return true;
            }
            else{
                return false;
            }
        }
        // -- 
        void setupBegin();
        // --     
        void OC_CAN_Transmit();
        void OC_CAN_Receive();
        void OC_CAN_send();
        
        void OC_test_CAN();

        void robot_forward();
        void robot_backward();
        void robot_stop();
        void robot_run();
        void loop();
        void debug();
};


#endif