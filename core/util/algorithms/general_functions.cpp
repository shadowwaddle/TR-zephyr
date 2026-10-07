#include "general_functions.h"
#include <cmath>

float calculateDeltaYaw(float curr_yaw, float des_yaw)
{
    float deltaYaw = des_yaw - curr_yaw;
    while (fabs(deltaYaw) > 180)
    {
        if (deltaYaw > 0)
            deltaYaw -= 360;
        else
            deltaYaw += 360;
    }
    return deltaYaw;
}

float capAngle(float curr_angle)
{
    if (fabsf(curr_angle) > 180.0f)
    {
        if (curr_angle > 0)
            curr_angle -= 360.0f;
        else
            curr_angle += 360.0f;
    }
    return curr_angle;
}

uint64_t now_us()
{
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

float degreesToRadians(float degrees) {
    return degrees * PI / 180.0f;
}

float radiansToDegrees(float radians)
{
    return radians / PI * 180.0f;
}