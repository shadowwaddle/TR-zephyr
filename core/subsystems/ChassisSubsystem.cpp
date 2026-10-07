#include "ChassisSubsystem.h"
#include "util/motor/DJIMotor.h"
#include <cmath>

/**
 * @param radius radius in meters
 */
ChassisSubsystem::ChassisSubsystem(const Config &config)
    : power_limit(60),
      LF(config.left_front_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      RF(config.right_front_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      LB(config.left_back_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      RB(config.right_back_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      encoder(config.encoder),
      yawPhase{config.yaw_initial_offset_ticks}, // change Yaw to CCW +, and ranges from 0 to 360
      imu(config.imu),
      chassis_radius(config.radius),
      FF_Ks(config.speed_pid_ff_ks)
{
    LF.outputCap = 16000; // DJIMotor class has a max outputCap: 16384
    RF.outputCap = 16000;
    LB.outputCap = 16000;
    RB.outputCap = 16000;

    setOmniKinematics(config.radius);
    m_OmniKinematicsLimits.max_Vel = MAX_VEL; // m/s
    m_OmniKinematicsLimits.max_vOmega = 8; // rad/s

    PEAK_POWER_ALL = 10000;
    PEAK_POWER_SINGLE = 8000;


//    LF.setSpeedPID(2, 0, 0);
//    RF.setSpeedPID(2, 0, 0);
//    LB.setSpeedPID(2, 0, 0);
//    RB.setSpeedPID(2, 0, 0);
    // LF.setSpeedPID(3, 0, 0);
    // RF.setSpeedPID(3, 0, 0);
    // LB.setSpeedPID(3, 0, 0);
    // RB.setSpeedPID(3 , 0, 0);

    pid_LF.setPID(3.38, 0, 0);
    pid_RF.setPID(3.38, 0, 0);
    pid_LB.setPID(3.38, 0, 0);
    pid_RB.setPID(3.38, 0, 0);
    pid_align.setPID(5, 0.0, 0.5);
    pid_align.setOutputCap(1000);
    yaw_velo_gain = 100;

    brakeMode = COAST;

    // isInverted[0] = 1; isInverted[1] = 1; isInverted[2] = 1; isInverted[3] = 1;
}

WheelSpeeds ChassisSubsystem::getWheelSpeeds() const
{
    return m_wheelSpeeds;
}

float ChassisSubsystem::limitAcceleration(float desiredRPM, float previousRPM, int power)
{
    float maxAccel = 100;
    float diff = desiredRPM - previousRPM;

    
    if ((desiredRPM > 0 && previousRPM < 0) || (desiredRPM < 0 && previousRPM > 0)) { // if robot trying to sudden change direction
        return 0;
    }
    
    
    if (diff > maxAccel){   // if the difference is greater than the max acceleration


        if(power == 0) {
            return desiredRPM; // let robot do its thing b/c it wont take power
        }

        return previousRPM + maxAccel;

    }
    else if (diff < -maxAccel) {
        if(power == 0) {
            return desiredRPM; // let robot do its thing b/c it wont take power
        }

        return previousRPM - maxAccel;
    }

    else {
        return desiredRPM; // under acceleration cap
    }
}

float ChassisSubsystem::p_theory(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm){
    float krpm2 = 0.000000000616869908524917;
    float kpwr2 = 2.8873053310419543e-26;
    float kboth = 0.00000000679867734389254;
    float a = 0.019247609510979;


    float p1 =  (kboth * LeftFrontPower * LeftFrontRpm) +  (krpm2 * LeftFrontRpm * LeftFrontRpm) +  (kpwr2 * LeftFrontPower * LeftFrontPower) + a;
    float p2 =  (kboth * RightFrontPower * RightFrontRpm) +  (krpm2 * RightFrontRpm * RightFrontRpm) +  (kpwr2 * RightFrontPower * RightFrontPower) + a;
    float p3 =  (kboth * LeftBackPower * LeftBackRpm) +  (krpm2 * LeftBackRpm * LeftBackRpm) +  (kpwr2 * LeftBackPower * LeftBackPower) + a;
    float p4 =  (kboth * RightBackPower * RightBackRpm) +  (krpm2 * RightBackRpm * RightBackRpm) +  (kpwr2 * RightBackPower * RightBackPower) + a;
    
    float p_tot = p1 + p2 + p3 + p4; 

    float A = 224.9;
    float B = 215.8;
    float C = 0.7955;

    float p_tot_c = (A * p_tot * p_tot) + (B * p_tot) + C;

    return p_tot_c;
}

float ChassisSubsystem::Bisection(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm, float chassisPowerLimit) {
    float scale = 0.5; // initial scale
    float precision = 0.25; // initial precision
    float powerInit = p_theory(LeftFrontPower, RightFrontPower, LeftBackPower, RightBackPower, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);


    if (powerInit > chassisPowerLimit) {

        float powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
        
        for (int i = 0; i < 6; i++) {
            
            // if (abs(powerScaled - chassisPowerLimit) < 0.1) {
            //     printf("ppppp \n");
            //     break;
            // }
            if (powerScaled > chassisPowerLimit) {
                scale = scale - precision;
                precision = precision / 2;
                powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
                // printf("powerScaled Down: %f\n", powerScaled);
            }

            else { // power is low enough
                scale = scale + precision;
                precision = precision / 2;
                powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
                // printf("powerScaled Up: %f\n", powerScaled);
            }
        }
        return scale;
    }

    else {
        return 1;
    }

}

float ChassisSubsystem::estimatePowerWatts(int torqueCounts)
{
    constexpr int   PEAK_TORQUE_COUNTS  = 5596;
    constexpr float SATURATION_RATIO    = 0.4375f;
    constexpr float SATURATION_CURRENT  = 1.22f;   // [A]
    constexpr float TORQUE_TO_AMP        = 0.3f;   // N*m / A From datasheet
    constexpr float BUS_VOLTAGE         = 24.0f;   // [V]

    float torque = std::abs(static_cast<float>(torqueCounts)) / PEAK_TORQUE_COUNTS;

    float currentA;
    if (torque > SATURATION_RATIO) {
        // Log an over-torque event at most once every 200 ms
        if ((now_us() - m_lastTorqueUs) / 1000UL > 200UL) {
            m_lastTorqueUs = now_us();
            currentA = SATURATION_CURRENT;
        }
        currentA = torque * TORQUE_TO_AMP;
    } else {
        currentA = torque * TORQUE_TO_AMP;
    }

    return BUS_VOLTAGE * currentA;
}

float ChassisSubsystem::setWheelSpeeds(WheelSpeeds wheelSpeeds)
{
    desiredWheelSpeeds = wheelSpeeds; // WheelSpeeds in RPM
    int powers[4] = {0,0,0,0};
    uint32_t time = now_us();


    powers[0] = motorPIDtoPower(LEFT_FRONT,wheelSpeeds.LF, (time - lastPIDTime));
    powers[1] = motorPIDtoPower(RIGHT_FRONT,wheelSpeeds.RF, (time - lastPIDTime));
    powers[2] = motorPIDtoPower(LEFT_BACK,wheelSpeeds.LB, (time - lastPIDTime));
    powers[3] = motorPIDtoPower(RIGHT_BACK,wheelSpeeds.RB, (time - lastPIDTime));
    

    float LFrpm = limitAcceleration(wheelSpeeds.LF, previousRPM[0], powers[0]);
    float RFrpm = limitAcceleration(wheelSpeeds.RF, previousRPM[1], powers[1]);
    float LBrpm = limitAcceleration(wheelSpeeds.LB, previousRPM[2], powers[2]);
    float RBrpm = limitAcceleration(wheelSpeeds.RB, previousRPM[3], powers[3]);

    // float LFrpm = wheelSpeeds.LF;
    // float RFrpm = wheelSpeeds.RF;
    // float LBrpm = wheelSpeeds.LB;
    // float RBrpm = wheelSpeeds.RB;

    
    // float LFrpm = limitAcceleration(wheelSpeeds.LF, previousRPM[0], LF.getData(POWEROUT));
    // float RFrpm = limitAcceleration(wheelSpeeds.RF, previousRPM[1], RF.getData(POWEROUT));
    // float LBrpm = limitAcceleration(wheelSpeeds.LB, previousRPM[2], LB.getData(POWEROUT));
    // float RBrpm = limitAcceleration(wheelSpeeds.RB, previousRPM[3], RB.getData(POWEROUT));

    previousRPM[0] = LFrpm;
    previousRPM[1] = RFrpm;
    previousRPM[2] = LBrpm;
    previousRPM[3] = RBrpm;

    // Seems to be somewhat  redundant to do PID twice; HOWEVER, we should profile exactly how long the limit acceleration takes

    // powers[0] = motorPIDtoPower(LEFT_FRONT,LFrpm, (time - lastPIDTime));
    // powers[1] = motorPIDtoPower(RIGHT_FRONT,RFrpm, (time - lastPIDTime));
    // powers[2] = motorPIDtoPower(LEFT_BACK,LBrpm, (time - lastPIDTime));
    // powers[3] = motorPIDtoPower(RIGHT_BACK,RBrpm, (time - lastPIDTime));
    // lastPIDTime = time;

    int p1 = abs(powers[0]);
    int p2 = abs(powers[1]);
    int p3 = abs(powers[2]);
    int p4 = abs(powers[3]);

    // Ignore Warning, we're casting to ints anyway so its fine
    int r1 = abs(getMotorSpeed(MotorLocation::LEFT_FRONT, RPM));
    int r2 = abs(getMotorSpeed(MotorLocation::RIGHT_FRONT, RPM));
    int r3 = abs(getMotorSpeed(MotorLocation::LEFT_BACK, RPM));
    int r4 = abs(getMotorSpeed(MotorLocation::RIGHT_BACK, RPM));

    float totalEstimatedWatts = estimatePowerWatts(LF.getData(TORQUE))
                               + estimatePowerWatts(RF.getData(TORQUE))
                               + estimatePowerWatts(LB.getData(TORQUE))
                               + estimatePowerWatts(RB.getData(TORQUE));

    // A small epsilon in the denominator prevents division by zero when idle.
    constexpr float POWER_MARGIN_W = 10.0f;
    float scale = std::min(1.0f, power_limit / (totalEstimatedWatts + POWER_MARGIN_W));
    
    LF.setPower(powers[0]*scale);
    RF.setPower(powers[1]*scale);
    LB.setPower(powers[2]*scale);
    RB.setPower(powers[3]*scale);

    p1 = fabsf(LF.getData(POWEROUT));
    p2 = fabsf(RF.getData(POWEROUT));
    p3 = fabsf(LB.getData(POWEROUT));
    p4 = fabsf(RB.getData(POWEROUT));

    return scale;
}

WheelSpeeds ChassisSubsystem::normalizeWheelSpeeds(WheelSpeeds wheelSpeeds) const
{
    float speeds[4] = {wheelSpeeds.LF, wheelSpeeds.RF, wheelSpeeds.LB, wheelSpeeds.RB};
    float max_speed = m_OmniKinematicsLimits.max_Vel;

    for (float speed : speeds)
        if (speed > max_speed)
            max_speed = speed;

    if (max_speed > m_OmniKinematicsLimits.max_Vel)
        for (float &speed : speeds)
            speed = speed / max_speed * m_OmniKinematicsLimits.max_Vel;

    return {speeds[0], speeds[1], speeds[2], speeds[3]};
}


void ChassisSubsystem::setWheelPower(WheelSpeeds wheelPower)
{
    setMotorPower(LEFT_FRONT, wheelPower.LF);
    setMotorPower(RIGHT_FRONT, wheelPower.RF);
    setMotorPower(LEFT_BACK, wheelPower.LB);
    setMotorPower(RIGHT_BACK, wheelPower.RB);
}

ChassisSpeeds ChassisSubsystem::getChassisSpeeds() const
{
    return m_chassisSpeeds;
}

float ChassisSubsystem::setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds_, DRIVE_MODE mode)
{
    float yawCurrent = 0;
    if (mode == REVERSE_YAW_ORIENTED)
    {
        yawCurrent = encoder->encoderMovingAverage();
        if (yawCurrent < 0.0f) {
            yawCurrent += 360.0f;
        }
        else if (yawCurrent > 360.0f) {
            yawCurrent -= 360.0f;
        }
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else if (mode == YAW_ORIENTED)
    {
        yawCurrent = encoder->encoderMovingAverage();
        if (yawCurrent < 0.0f) {
            yawCurrent += 360.0f;
        }
        else if (yawCurrent > 360.0f) {
            yawCurrent -= 360.0f;
        }
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else if (mode == ROBOT_ORIENTED)
    {
        desiredChassisSpeeds = desiredChassisSpeeds_; // ChassisSpeeds in m/s
    }
    else if (mode == ODOM_ORIENTED) 
    {
        yawCurrent = encoder->encoderMovingAverage();
        if (yawCurrent < 0.0f) {
            yawCurrent += 360.0f;
        }
        else if (yawCurrent > 360.0f) {
            yawCurrent -= 360.0f;
        }

        float yawDelta = yawOdom - yawCurrent;
        float imuDelta = imuOdom - imuAngles.yaw;
        float delta = imuDelta - yawDelta;
        float del = yawOdom + delta;
        while (del > 360.0f) del -= 360;
        while (del < 0) del += 360;
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawOdom + delta);
    }
    else if (mode == YAW_ALIGN)
    {
        yawCurrent = encoder->encoderMovingAverage();
        if (yawCurrent < 0.0f) {
            yawCurrent += 360.0f;
        }
        else if (yawCurrent > 360.0f) {
            yawCurrent -= 360.0f;
        }

        // Compute yaw error(how much the yaw needs to recorrect)
        float yawError = (yawCurrent - yawPhase);
        while (yawError > 180) yawError -= 360;
        while (yawError < -180) yawError += 360;
        
        if (fabsf(yawError) < 5) yawError = 0;

        if (yawError > 90) yawError -= 180;
        else if (yawError < -90) yawError += 180;

        float yaw_velo = (yawCurrent - yawPrior);
        float deg2rad = PI/180; // convert to rad and just run at 2x that rad/s
        pid_align.feedForward = yaw_velo * yaw_velo_gain;
        float omegaCmd = pid_align.calculatePeriodic(yawError, 1000) * deg2rad;

        if (fabsf(omegaCmd) < 0.1f) omegaCmd = 0;

        ChassisSpeeds xAlignSpeeds = {desiredChassisSpeeds_.vX, desiredChassisSpeeds_.vY, omegaCmd};
        desiredChassisSpeeds = rotateChassisSpeed(xAlignSpeeds, yawCurrent);
    }
    yawPrior = encoder->encoderMovingAverage();
    
    WheelSpeeds wheelSpeeds = chassisSpeedsToWheelSpeeds(desiredChassisSpeeds); // in m/s (for now)
    wheelSpeeds = normalizeWheelSpeeds(wheelSpeeds);
    wheelSpeeds *= (1 / (WHEEL_DIAMETER_METERS / 2) / (2 * PI / 60) * M3508_GEAR_RATIO);
    float scale = setWheelSpeeds(wheelSpeeds);
    return scale;
}

/**
 * There's no setChassisPower because it doesn't make sense.
 * Power (is not PWM voltage) saturates your motor speeds, and it's not related to motor speed. 
 */

ChassisSpeeds ChassisSubsystem::rotateChassisSpeed(ChassisSpeeds speeds, float yawCurrent)
{
    // rotate angle counter clockwise
    float theta = (yawCurrent - yawPhase) / 180 * PI;
    return {speeds.vX * cosf(theta) - speeds.vY * sinf(theta),
            speeds.vX * sinf(theta) + speeds.vY * cosf(theta),
            speeds.vOmega};
}

DJIMotor &ChassisSubsystem::getMotor(MotorLocation location)
{
    switch (location)
    {
    case LEFT_FRONT:
        return LF;
    case RIGHT_FRONT:
        return RF;
    case LEFT_BACK:
        return LB;
    case RIGHT_BACK:
        return RB;
    }
    
    assert(false && "Invalid MotorLocation");
    __builtin_unreachable();
}

void ChassisSubsystem::setMotorSpeedPID(MotorLocation location, float kP, float kI, float kD)
{
    getMotor(location).setSpeedPID(kP, kI, kD);
}

void ChassisSubsystem::setSpeedIntegralCap(MotorLocation location, float cap)
{
    getMotor(location).setSpeedIntegralCap(cap);
}

void ChassisSubsystem::setSpeedFeedforward(MotorLocation location, float FF)
{
    // getMotor(location).pidSpeed.feedForward = FF * INT15_T_MAX;
    if(location == LEFT_FRONT){
        pid_LF.feedForward = FF * INT15_T_MAX;
    }else if(location == LEFT_BACK){
        pid_LB.feedForward = FF * INT15_T_MAX;
    }else if(location == RIGHT_FRONT){
        pid_RF.feedForward = FF * INT15_T_MAX;
    }else if(location == RIGHT_BACK){
        pid_RB.feedForward = FF * INT15_T_MAX;
    }
}


ChassisSubsystem::BrakeMode ChassisSubsystem::getBrakeMode()
{
    return brakeMode;
}

void ChassisSubsystem::setBrakeMode(BrakeMode brakeMode)
{
    this->brakeMode = brakeMode;
}

void ChassisSubsystem::initializeImu()
{
    return;
    // imu.set_mounting_position(MT_P1);
}

float ChassisSubsystem::getMotorSpeed(MotorLocation location, SPEED_UNIT unit = RPM)
{
    float speed = getMotor(location).getData(VELOCITY);
    switch (unit)
    {
    case RPM:
        return (speed * M3508_GEAR_RATIO) / (2 * PI / 60);
    case METER_PER_SECOND:
        return speed * (WHEEL_DIAMETER_METERS / 2);
    case RAD_PER_SECOND:
        return speed;
    }

    assert(false && "Invalid motor speed unit");
    __builtin_unreachable();
}

void ChassisSubsystem::readImu()
{
    imu.getImuAngles();
}

void ChassisSubsystem::periodic(IMU::EulerAngles *imuCurr)
{   
    imuAngles.yaw = imuCurr->yaw;
    imuAngles.pitch = imuCurr->pitch;
    imuAngles.roll = imuCurr->roll;
    m_wheelSpeeds = {getMotorSpeed(LEFT_FRONT, METER_PER_SECOND), getMotorSpeed(RIGHT_FRONT, METER_PER_SECOND),
                     getMotorSpeed(LEFT_BACK, METER_PER_SECOND), getMotorSpeed(RIGHT_BACK, METER_PER_SECOND)};

    m_chassisSpeeds = wheelSpeedsToChassisSpeeds(m_wheelSpeeds);
}


// Moved to general functions
// float ChassisSubsystem::degreesToRadians(float degrees)
// {
//     return degrees * PI / 180.0;
// }

// float ChassisSubsystem::radiansToDegrees(float radians)
// {
//     return radians / PI * 180.0;
// }

int ChassisSubsystem::getHeadingDegrees() const
{
    return (int)imuAngles.yaw;
}

void ChassisSubsystem::setOmniKinematicsLimits(float max_Vel, float max_vOmega)
{
    m_OmniKinematicsLimits.max_Vel = max_Vel;
    m_OmniKinematicsLimits.max_vOmega = max_vOmega;
}

//NEW STUFF FROM THIS PAPER: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

// void ChassisSubsystem::setOmniKinematics(float radius)
// {
//     float SQRT_2 = sqrt(2);
//     m_OmniKinematics.r1x = -sqrt(radius);
//     m_OmniKinematics.r1y = sqrt(radius);

//     m_OmniKinematics.r2x = sqrt(radius);
//     m_OmniKinematics.r2y = sqrt(radius);

//     m_OmniKinematics.r3x = -sqrt(radius);
//     m_OmniKinematics.r3y = -sqrt(radius);

//     m_OmniKinematics.r4x = sqrt(radius);
//     m_OmniKinematics.r4y = -sqrt(radius);
// }

void ChassisSubsystem::setOmniKinematics(float radius, HOLONOMIC_MODE mode)
{
    if (mode == OMNI) {

        float SQRT_2 = sqrt(2);
        
        m_OmniKinematics.r1x = radius/SQRT_2;
        m_OmniKinematics.r1y = radius/SQRT_2;

        m_OmniKinematics.r2x = radius/SQRT_2;
        m_OmniKinematics.r2y = radius/SQRT_2;

        m_OmniKinematics.r3x = radius/SQRT_2;
        m_OmniKinematics.r3y = radius/SQRT_2;

        m_OmniKinematics.r4x = radius/SQRT_2;
        m_OmniKinematics.r4y = radius/SQRT_2;
    } 

    else if (mode == MECANUM) {
        m_OmniKinematics.r1x = WHEEL_TO_CHASSIS_CENTER_LY;
        m_OmniKinematics.r1y = WHEEL_TO_CHASSIS_CENTER_LX;

        m_OmniKinematics.r2x = WHEEL_TO_CHASSIS_CENTER_LY;
        m_OmniKinematics.r2y = WHEEL_TO_CHASSIS_CENTER_LX;

        m_OmniKinematics.r3x = WHEEL_TO_CHASSIS_CENTER_LY;
        m_OmniKinematics.r3y = WHEEL_TO_CHASSIS_CENTER_LX;

        m_OmniKinematics.r4x = WHEEL_TO_CHASSIS_CENTER_LY;
        m_OmniKinematics.r4y = WHEEL_TO_CHASSIS_CENTER_LX;
    }
}

//inputs chassis speeds in m/s, outputs wheel speeds in m/s
WheelSpeeds ChassisSubsystem::chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds)
{
    return {(+chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r1x) + (m_OmniKinematics.r1y))),
            (-chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y))),
            (+chassisSpeeds.vX + chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y))),
            (-chassisSpeeds.vX + chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r4x) + (m_OmniKinematics.r4y)))};
}

// ChassisSpeeds ChassisSubsystem::wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds)
// {
//     Eigen::MatrixXd Inv_K(4, 3);
//     float coef = 1 / sqrt(2);
//     Inv_K << coef, coef, coef * ((m_OmniKinematics.r1x) - (m_OmniKinematics.r1y)),
//         coef, -coef, -coef * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y)),
//         -coef, coef, coef * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y)),
//         -coef, -coef, -coef * ((m_OmniKinematics.r4x) - (m_OmniKinematics.r4y));
//     Eigen::MatrixXd FWD_K(3, 4);
//     FWD_K = Inv_K.completeOrthogonalDecomposition().pseudoInverse();
//     // printf("[%.1f %.1f %.1f %.1f\n%.1f %.1f %.1f %.1f\n%.1f %.1f %.1f %.1f]\n", 
//     //     FWD_K(0,0), FWD_K(0,1), FWD_K(0,2), FWD_K(0,3), 
//     //     FWD_K(1,0), FWD_K(1,1), FWD_K(1,2), FWD_K(1,3), 
//     //     FWD_K(2,0), FWD_K(2,1), FWD_K(2,2), FWD_K(2,3));
//     Eigen::MatrixXd WS(4, 1);
//     WS << wheelSpeeds.LF, wheelSpeeds.RF, wheelSpeeds.LB, wheelSpeeds.RB;
//     Eigen::MatrixXd CS(3, 1);
//     CS = FWD_K * WS;
//     return {CS(0, 0), CS(1, 0), CS(2, 0)};
// }

ChassisSpeeds ChassisSubsystem::wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds)
{
    float dist = chassis_radius/sqrtf(2);
    float vX = (wheelSpeeds.LF + wheelSpeeds.RF - wheelSpeeds.LB - wheelSpeeds.RB) / 4;
    float vY = (wheelSpeeds.LF - wheelSpeeds.RF + wheelSpeeds.LB - wheelSpeeds.RB) / 4;
    float vOmega = (-wheelSpeeds.LF - wheelSpeeds.RF - wheelSpeeds.LB - wheelSpeeds.RB) / (4*(2 * dist));
    return {vX, vY, vOmega};
}


void ChassisSubsystem::setMotorPower(MotorLocation location, float power)
{
    if (brakeMode == BRAKE && power == 0) // Should be BRAKE
    {
        getMotor(location).setSpeed(0);
        setSpeedFeedforward(location, 0);
        return;
    }
    getMotor(location).setPower(power * INT15_T_MAX);
}

void ChassisSubsystem::setMotorSpeedRPM(MotorLocation location, float speed)
{
    if (brakeMode == COAST && speed == 0)
    {
        setMotorPower(location, 0);
        setSpeedFeedforward(location, 0);
        return;
    }
    getMotor(location).setSpeed(speed);
    float sgn_speed = speed / fabsf(speed); // if speed is 0, it won't execute this line
    setSpeedFeedforward(location, FF_Ks * sgn_speed);
}

int ChassisSubsystem::motorPIDtoPower(MotorLocation location, float speed, uint32_t dt)
{
    if (brakeMode == COAST && speed == 0)
    {
        setSpeedFeedforward(location, 0);
        return 0;
    }
    
    int power = 0;
    PID* pids[4] = {&pid_LF, &pid_RF, &pid_LB, &pid_RB};
    
    power = pids[location]->calculate(speed, getMotorSpeed(location, RPM), dt);
    // printf("[%d]",power);

    if(speed == 0) {
        setSpeedFeedforward(location, 0);
        return power;
    }
    float sgn_speed = speed / fabsf(speed); // if speed is 0, it won't execute this line
    setSpeedFeedforward(location, FF_Ks * sgn_speed);
    return power;
}


bool ChassisSubsystem::setOdomReference() {
    // yawOdom = -(1.0 - (float(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0;
    imuOdom = imuAngles.yaw;
    return true;
}

// float ChassisSubsystem::getEncoderYawPosition() {
//     if (encoder == nullptr) {
//         return -1.0f;  // Encoder not available
//     }

//     static float filtered_yaw = 0.0f;
//     float filter_alpha = 0.2f;  // 0.0-1.0: lower = more smoothing, higher = more responsive

//     float duty_raw = 360 - encoder->dutycycle();
//     float duty_min = 0.02943f;   // 2.943%
//     float duty_max = 0.97058f;   // 97.058%

//     //low pass filter 
//     float yaw_position = (float)(abs(((duty_raw - duty_min) / (duty_max - duty_min)) * 360.0));
//     filtered_yaw = filtered_yaw * (1.0f - filter_alpha) + yaw_position *  filter_alpha;
//     // printf("%.2f\n",yaw_position);
//     return filtered_yaw;
// }

// float ChassisSubsystem::encoderMovingAverage() {
//     const int windowSize = 20;
//     static float readings[windowSize] = {0};
//     static int index = 0;
//     static bool filled = false;

//     float newReading = getEncoderYawPosition();
//     if (newReading < 0) {
//         return -1.0f; // Encoder not available
//     }

//     readings[index] = newReading;
//     index = (index + 1) % windowSize;
//     if (index == 0) {
//         filled = true;
//     }

//     float sum = 0.0;
//     float result = 0.0;

//     if (filled) {
//         for (int i = 0; i < windowSize; i++) {
//             sum += readings[i];
//         }
//         result = sum / windowSize;
//     } else {
//         for (int i = 0; i < index; i++) {
//             sum += readings[i];
//         }
//         result = sum / index;
//     }
//     // printf("%.2f\n",result);
//     return result;

// }

void ChassisSubsystem::updateYawPhaseFromEncoder() {
    float encoder_reading = 360 - encoder->encoderMovingAverage();
        if (encoder_reading < 0.0f) {
            encoder_reading += 360.0f;
        }
        else if (encoder_reading > 360.0f) {
            encoder_reading -= 360.0f;
        }
    if (encoder_reading >= 0) {
        yawPhase = encoder_reading;
    }
}

float ChassisSubsystem::radiansToTicks(float radians)
{
    return radians / (2 * PI) * TICKS_PER_ROTATION;
}

float ChassisSubsystem::ticksToRadians(float ticks)
{
    return ticks / TICKS_PER_ROTATION * (2 * PI);
}

float ChassisSubsystem::rpmToRadPerSecond(float RPM)
{
    return RPM * (2 * PI) / SECONDS_PER_MINUTE;
}

float ChassisSubsystem::radPerSecondToRPM(float radPerSecond)
{
    return radPerSecond / (2 * PI) * SECONDS_PER_MINUTE;
}