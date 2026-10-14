#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <util/peripherals/imu/ISM330.h>


static const struct spi_dt_spec imu_spec1 = SPI_DT_SPEC_GET(DT_NODELABEL(imu1), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);
//static const struct spi_dt_spec imu_spec2 = SPI_DT_SPEC_GET(DT_NODELABEL(imu2), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);

IMU::EulerAngles imuAngles1;
//IMU::EulerAngles imuAngles2;

ISM330 imu1_(imu_spec1);
//ISM330 imu2_(imu_spec2);



void periodic() {
    imu1_.mahonyUpdateIMU(200 / 1000.0);
    //imu2_.mahonyUpdateIMU(200 / 1000.0);

    auto accel1 = imu1_.readAccel();
    auto gyro1 = imu1_.readGyro();
    imuAngles1 = imu1_.getImuAngles();

    // auto accel2 = imu2_.readAccel();
    // auto gyro2 = imu2_.readGyro();
    // imuAngles2 = imu2_.getImuAngles();

   printk("IMU 1 WHO_AM_I: 0x%02X\n", imu1_.getWhoAmI());
   // printk("IMU 2 WHO_AM_I: 0x%02X\n", imu2_.getWhoAmI());

    printk("%.2f, %.2f, %.2f\n", static_cast<double>(imuAngles1.roll), static_cast<double>(imuAngles1.pitch), static_cast<double>(imuAngles1.yaw));
    printk("Accel X: %.2f\n", static_cast<double>(accel1.x));
    printk("Gyro Y: %.2f\n", static_cast<double>(gyro1.y));

    // printk("%.2f, %.2f, %.2f\n", static_cast<double>(imuAngles2.roll), static_cast<double>(imuAngles2.pitch), static_cast<double>(imuAngles2.yaw));
    // printk("Accel X: %.2f\n", static_cast<double>(accel2.x));
    // printk("Gyro Y: %.2f\n", static_cast<double>(gyro2.y));
}

int main(void)
{
    printk("HELLO\n");
    

    if (device_is_ready(imu_spec1.bus)) {
        printk("IMU devices are ready\n");
    } else {
        printk("IMU devices are not ready\n");
    }

    if (!imu1_.begin(0.9, 0)) {
        printk("IMU initialization failed!\n");
        return -1;
    }
    
    while (true) {
        periodic();
        k_sleep(K_MSEC(200));
    }
}
