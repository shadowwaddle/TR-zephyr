#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <util/peripherals/imu/ISM330.h>

static const struct spi_dt_spec imu_spec = SPI_DT_SPEC_GET(DT_NODELABEL(imu), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);

IMU::EulerAngles imuAngles;
ISM330 imu_(imu_spec);



void periodic() {
    imu_.mahonyUpdateIMU(200 / 1000.0);
    auto accel = imu_.readAccel();
    auto gyro = imu_.readGyro();
    imuAngles = imu_.getImuAngles();

    printk("%.2f, %.2f, %.2f\n", static_cast<double>(imuAngles.roll), static_cast<double>(imuAngles.pitch), static_cast<double>(imuAngles.yaw));
    printk("Accel X: %.2f\n", static_cast<double>(accel.x));
    printk("Gyro Y: %.2f\n", static_cast<double>(gyro.y));
}

int main(void)
{
    printk("HELLO\n");

    if (device_is_ready(imu_spec.bus)) {
        printk("IMU device is ready\n");
    } else {
        printk("IMU device is not ready\n");
    }

    if (!imu_.begin(0.9, 0)) {
        printk("IMU initialization failed!\n");
        return -1;
    }
    
    while (true) {
        periodic();
        k_sleep(K_MSEC(200));
    }
}
