# Explicitly name it because otherwise Zephyr appends _ to the beginning because we're up a level on the directory
zephyr_library_named(core)
zephyr_library_sources(
    # Algo Section
    util/algorithms/general_functions.cpp
    util/algorithms/PID.cpp
    util/algorithms/ChassisSpeeds.cpp
    util/algorithms/mbedMutex.cpp
    util/algorithms/WheelSpeeds.cpp

    # Comms Section
    util/communications/CANMsg.h
    util/communications/CANHandler.h
    util/communications/DJIRemote2.cpp
    util/communications/ui/ui_g.cpp
    
    util/communications/referee/ref_serial.cpp
    util/communications/referee/crc.cpp
    util/communications/mbedSerial.cpp
    util/communications/DMAUart.cpp
    
    util/communications/jetson/Jetson.cpp
    
    # Misc Util
    util/motor/DJIMotor.cpp
    util/peripherals/imu/ISM330.cpp
    util/peripherals/encoder/MA4.cpp

    # Subsystems
    subsystems/ChassisSubsystem.cpp
    subsystems/TurretSubsystem.cpp
    subsystems/ShooterSubsystem.cpp
    subsystems/HeroShooterSubsystem.cpp

)
zephyr_library_include_directories(
    .
    util/algorithms
    util/communications
    util/motor
    util/peripherals
    subsystems
)