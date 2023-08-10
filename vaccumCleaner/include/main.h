#ifndef MAIN_H
#define MAIN_H

// all things relevant to robot and manipulate robot
class Robot
{
    private:
        
    public:
    // Navigate_robot();
    // ~Navigate_robot();
    void run(int Motor_speeda, int Motor_speedb);
    void spin(int direction, int motor_speed);
    void stop();

};

// all things relevant to ESP32 controller
class ESP32_Controller
{
    private:
        Robot* rb_ctrl;
    public:
        // ESP32_Controller();
        // ~ESP32_Controller();
        void pin_init();
        void pwm_pin_init();
        void launch_init();

};

#endif