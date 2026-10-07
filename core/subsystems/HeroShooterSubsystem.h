

#pragma once

#include <util/algorithms/PID.h>
#include <util/communications/CANHandler.h>
#include <util/motor/DJIMotor.h>
#include "Subsystem.h"
#include "ShooterSubsystem.h"

// struct for config
class HeroShooterSubsystem : public Subsystem
{
public:
    struct config
    {
        const struct device *top_can_device;
        CANHandler::CANBus canBusTopFeed;

        const struct device *indexer_can_device;
        CANHandler::CANBus canBusIndexer;

        int heat_limit;

        short flywheelL_id;
        short flywheelR_id;
        short indexer_id;
        short feeder_id;

        PID::config flywheelL_PID;
        PID::config flywheelR_PID;
        PID::config feeder_PID;
        PID::config indexer_PID_vel;
        PID::config indexer_PID_pos;


        bool invert = false;
    };

    HeroShooterSubsystem(config configuration);

    void setState(ShootState shoot_state);

    void periodic(int curr_heat, int heat_limit);
    DJIMotor flywheelL, flywheelR, indexer, feeder;

private:
    static unsigned long shooter_time;
    bool invert_flywheel;



    ShootState shoot;
    
    
    int barrel_heat;
    int barrel_heat_limit;
    bool shootReady;

    int shootTargetPosition;
};
