#include <zephyr/kernel.h>
#include <stdlib.h>
#include <cmath>

#define PI 3.14159265f

float calculateDeltaYaw(float curr_yaw, float des_yaw);

float capAngle(float curr_angle);

float degreesToRadians(float degrees);

float radiansToDegrees(float radians);


/**
* @brief Returns the current microsecond ticks
*/
uint64_t now_us(); 
