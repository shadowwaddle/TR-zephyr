#pragma once

// #include "PinNames.h"
// #include "mbed.h"
#include <zephyr/kernel.h>
#include "stm32f446xx.h"
#include "util/communications/DJIRemote2.h"
#include "util/communications/referee/ref_serial.h"
#include <util/communications/mbedDigitalOut.cpp>
#include "util/motor/DJIMotor.h"
#include <subsystems/ChassisSubsystem.h>
#include <subsystems/TurretSubsystem.h>
#include <subsystems/ShooterSubsystem.h>

class BaseRobot {
  public:
    struct Config {
        // PinName remote_tx_pin = NC; 
		// PinName remote_rx_pin = PA_10;

        const struct device *controller_uart_dev; 
        
        // PinName referee_tx_pin = PC_10;
        // PinName referee_rx_pin = PC_11;

        const struct device *referee_uart_dev;
        USART_TypeDef *referee_usart_type;

        // PinName can1_rx_pin = PA_11;
        // PinName can1_tx_pin = PA_12;
        const struct device *can1_dev;

        // PinName can2_rx_pin = PB_12;
        // PinName can2_tx_pin = PB_13;
        const struct device *can2_dev;

        // PinName led0_pin = PB_0;
        // PinName led1_pin = PC_1;
        // PinName led2_pin = PC_0;
        const struct gpio_dt_spec *led0_dev;
        const struct gpio_dt_spec *led1_dev;
        const struct gpio_dt_spec *led2_dev;
    };

    DJIRemote2 remote_;
    Referee referee_;

    CANHandler canHandler1_;
    CANHandler canHandler2_;
    DigitalOut led0_;
    DigitalOut led1_;
    DigitalOut led2_;
    

    // Remote control variables
    float scalar = 1;
    float jx = 0; // -1 to 1
    float jy = 0; // -1 to 1
    // Pitch, Yaw
    float jpitch = 0; // -1 to 1
    float jyaw = 0; // -1 to 1
    float myaw = 0;
    float mpitch = 0;
    int pitchVelo = 0;
    // joystick tolerance
    double tolerance = 0.05;
    // Keyboard Driving
    float mult = 0.7;
    float omega_speed = 0;
    float max_linear_vel = 0;

    // drive and shooting mode
    // "o" - joystick
    // "u" - drive
    // "d" - beyblade
    // "m" - off
    // "y" - yaw-align

    // "o" - joystick
    // "d" - flywheel
    // "m" - flywheel off
    char drive = 'o'; //default o when using joystick 
    char shot = 'o'; //default o when using joystick
    bool cv_enabled_ = false;

    // clang-format off
    BaseRobot(const Config &config)
        : remote_(config.controller_uart_dev),
          referee_(config.referee_uart_dev, config.referee_usart_type), 
          canHandler1_(config.can1_dev),
          canHandler2_(config.can2_dev),
         led0_(*config.led0_dev),
         led1_(*config.led1_dev),
         led2_(*config.led2_dev)
    {};
    // clang-format on

    virtual ~BaseRobot() = default;

    virtual void init() = 0;
    virtual void periodic(const unsigned long dt_us) = 0;
    virtual void end_of_loop() {};

    // default 1000hz main loop. Can be overriden
    virtual unsigned int main_loop_dt_ms() { return 1; };

    virtual void main_loop() {
        unsigned long loop_clock_us = now_us();
        unsigned long prev_loop_time_us = loop_clock_us;
        unsigned long prev_remote_time_us = loop_clock_us;

        unsigned long main_loop_dt_ms = this->main_loop_dt_ms();

        // Init all constants, subsystems, sensors, IO, etc.
        // Each can message has an id and data, and djimotor ids start from 0x201(m3508 id 1) and
        // end at 0x20D (gm6020 id 8), there is an overlap of 4 motors (M3508 id 5-8 and gm6020 id
        // 1-4), so that is why we have 12 values only
        // canHandler1_.registerCallback(0x201, 0x20D, DJIMotor::getCan1Feedback);
        // canHandler2_.registerCallback(0x201, 0x20D, DJIMotor::getCan2Feedback);

        canHandler1_.registerCallback(0x201,0x20D, DJIMotor::getCanOneFeedback);
        canHandler2_.registerCallback(0x201, 0x20D, DJIMotor::getCanTwoFeedback);

        DJIMotor::setCanHandlers();

        CANHandler *can1 = DJIMotor::getCanHandler(CANHandler::CANBUS_1);
        CANHandler *can2 = DJIMotor::getCanHandler(CANHandler::CANBUS_2);
        if (can1) can1->registerCallback(0x201, 0x20D, DJIMotor::getCanOneFeedback);
        if (can2) can2->registerCallback(0x201, 0x20D, DJIMotor::getCanTwoFeedback);

        init();

        while (true) {
            loop_clock_us = now_us();

            // 20 ms remote read
            if ((loop_clock_us - prev_remote_time_us) / 1000 >= 15) {
                remote_.update();
                remoteRead();
                prev_remote_time_us = loop_clock_us;
            }

            if ((loop_clock_us - prev_loop_time_us) / 1000 >= main_loop_dt_ms) {
                // Add subsystems in periodic
                led0_.toggle();

                periodic(loop_clock_us - prev_loop_time_us);
                prev_loop_time_us = loop_clock_us;

                // Motor updates
                DJIMotor::sendValues();
            }
            // Add sensors updates in your end of loop
            end_of_loop();

            // canHandler1_.readAllCan();
            // canHandler2_.readAllCan();
            if (can1) can1->readAllCan();
            if (can2) can2->readAllCan();

            k_sleep(K_MSEC(10));
        }
    }

    void remoteRead()
    {
        //Keyboard-based drive and shoot mode
        if(remote_.keyPressed(DJIRemote2::Key::R)){
            drive = 'm';
        }else if(remote_.keyPressed(DJIRemote2::Key::E)){
            drive = 'u';
        }else if(remote_.keyPressed(DJIRemote2::Key::Q)){
            drive = 'd';        
        }else if(remote_.keyPressed(DJIRemote2::Key::B)){
            drive = 'y';
        }

        if(remote_.keyPressed(DJIRemote2::Key::V)){
            shot = 'm';
        }else if(remote_.keyPressed(DJIRemote2::Key::C)){
            shot = 'd';        
        }else if(remote_.keyPressed(DJIRemote2::Key::Z)){
            shot = 'z';
        }
        
        if(remote_.getMouseR() || remote_.getDialValue() == -1){
            cv_enabled_ = true;
        }else if(!remote_.getMouseR() ){
            cv_enabled_ = false;
        }

        //Driving input
        scalar = 1;
        jx = -remote_.getJoystickValue(DJIRemote2::Joystick::LEFT_HORIZONTAL) * scalar; // -1 to 1
        jy = remote_.getJoystickValue(DJIRemote2::Joystick::LEFT_VERTICAL) * scalar; // -1 to 1
        //Pitch, Yaw
        jpitch = remote_.getJoystickValue(DJIRemote2::Joystick::RIGHT_VERTICAL) * scalar; // -1 to 1
        jyaw = remote_.getJoystickValue(DJIRemote2::Joystick::RIGHT_HORIZONTAL) * scalar; // -1 to 1

        myaw = remote_.getMouseX();
        mpitch = remote_.getMouseY();
        // redundant since applydeadzone does this anyway
        // jx = (fabs(jx) < tolerance) ? 0 : jx;
        // jy = (fabs(jy) < tolerance) ? 0 : jy;
        // jpitch = (fabs(jpitch) < tolerance) ? 0 : jpitch;
        // jyaw = (fabs(jyaw) < tolerance) ? 0 : jyaw;

        

        // Shift to make robot go slower
        if (remote_.keyPressed(DJIRemote2::Key::SHIFT)) {
            mult = 0.5;
        }
        if(remote_.keyPressed(DJIRemote2::Key::CTRL)){
            mult = 1;
        }

        jx += mult * ((remote_.keyPressed(DJIRemote2::Key::D) ? -1 : 0) + (remote_.keyPressed(DJIRemote2::Key::A) ? 1 : 0));
        jy += mult * ((remote_.keyPressed(DJIRemote2::Key::W) ? 1 : 0) + (remote_.keyPressed(DJIRemote2::Key::S) ? -1 : 0));

        float j_hypo = sqrtf(jx * jx + jy * jy);
        if(j_hypo > 1.0f){
            jx = jx / j_hypo;
            jy = jy / j_hypo;
        }
        //Bounding the four j variables
        jx = fmaxf(-1.0F, fminf(1.0f, jx));
        jy = fmaxf(-1.0F, fminf(1.0f, jy));
        jpitch = fmaxf(-1.0F, fminf(1.0f, jpitch));
        jyaw = fmaxf(-1.0F, fminf(1.0f, jyaw));

        // max_linear_vel = -1.24 + 0.0513 * chassis.power_limit + -0.000216 * (chassis.power_limit * chassis.power_limit);
        // float max_omega = 0.326 + 0.0857 * chassis_power_limit + -0.000183 * (chassis_power_limit * chassis_power_limit);
        float max_omega = 4.8f;

        if(remote_.keyPressed(DJIRemote2::Key::CTRL)){
            jx = 0.0f;
            jy = 0.0f;
            max_omega = 6.1f;
        }

        float linear_hypo = sqrtf(jx * jx + jy * jy);
        if(linear_hypo > 0.8f){
            linear_hypo = 0.8f;
        }

        float available_beyblade = 1.0f - linear_hypo;
        omega_speed = max_omega * available_beyblade;
    }
};