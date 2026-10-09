// #include "stm32f446xx.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <util/communications/DJIRemote2.h>
#include <util/communications/CANHandler.h>
#include <util/communications/jetson/Jetson.h>
#include <util/peripherals/imu/ISM330.h>
#include <util/algorithms/PID.h>
#include <base_robot/BaseRobot.h>
#include <zephyr/dt-bindings/pwm/pwm.h>



// Devices from DT
const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

static const struct i2c_dt_spec imu_spec = I2C_DT_SPEC_GET(DT_NODELABEL(imu));



IMU::EulerAngles imuAngles;

ISM330 imu_(imu_spec);
bool imu_initialized{false};

double AG[6];

void periodic() {
    imu_.mahonyUpdateIMU(200 / 1000.0);
    imuAngles = imu_.getImuAngles();
  
    // auto imuaccel = imu_.readAccel();
    
    // printk("Accel: %.2f, %.2f, %.2f\n", static_cast<double>(imuaccel.x), static_cast<double>(imuaccel.y), static_cast<double>(imuaccel.z));

    printk("%.2f, %.2f, %.2f\n", static_cast<double>(imuAngles.roll), static_cast<double>(imuAngles.pitch), static_cast<double>(imuAngles.yaw));
}

int main(void)
{
    printk("HELLO\n");


    if (device_is_ready(imu_spec.bus)) {
        printk("IMU device is ready\n");
    } else {
        printk("IMU device is not ready\n");
    }

    i2c_recover_bus(imu_spec.bus);
    imu_.begin(0.9, 0);
    
    while (true) {
        periodic();
        k_sleep(K_MSEC(200));
    }
}
