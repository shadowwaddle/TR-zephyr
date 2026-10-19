#include "stm32f4xx.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <util/peripherals/imu/ISM330.h>


static const struct spi_dt_spec imu_spec = SPI_DT_SPEC_GET(DT_NODELABEL(imu1), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);
static const struct spi_dt_spec mcu_spec = SPI_DT_SPEC_GET(DT_NODELABEL(mcu2), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);
IMU::EulerAngles imuAngles1;

ISM330 imu_(imu_spec);

// spi helper transfer and recieves 1 byte
uint8_t transferSPI(uint8_t data) {
    uint8_t rx = 0;
    struct spi_buf tx_buf = {
        .buf = &data,
        .len = 1
    };
    struct spi_buf rx_buf = {
        .buf = &rx,
        .len = 1
    };
    struct spi_buf_set tx_set = {
        .buffers = &tx_buf,
        .count = 1
    };
    struct spi_buf_set rx_set = {
        .buffers = &rx_buf,
        .count = 1
    };
    int ret = spi_transceive_dt(&mcu_spec, &tx_set, &rx_set);

    if (ret < 0) {
        printk("SPI transfer failed: %d\n", ret);
    }
    return rx;
}

void periodic() {
    // IMU readings
    imu_.mahonyUpdateIMU(200 / 1000.0);

    auto accel1 = imu_.readAccel();
    auto gyro1 = imu_.readGyro();
    imuAngles1 = imu_.getImuAngles();

    printk("IMU 1 WHO_AM_I: 0x%02X\n", imu_.getWhoAmI());

    printk("%.2f, %.2f, %.2f\n",
        static_cast<double>(imuAngles1.roll),
        static_cast<double>(imuAngles1.pitch),
        static_cast<double>(imuAngles1.yaw));

    printk("Accel X: %.2f\n", static_cast<double>(accel1.x));
    printk("Gyro Y: %.2f\n", static_cast<double>(gyro1.y));

    // SPI communication with STM32 #2
    uint8_t response = transferSPI(0x55);
    printk("MCU Response: 0x%02X\n", response);
}

int main(void)
{
    printk("HELLO\n");

    if (!device_is_ready(imu_spec.bus)) {
        printk("IMU SPI not ready!\n");
        return -1;
    }

    if (!spi_is_ready_dt(&mcu_spec)) {
        printk("MCU SPI not ready!\n");
        return -1;
    }

    if (!imu_.begin(0.9, 0)) {
        printk("IMU initialization failed!\n");
        return -1;
    }

    while (true) {
        periodic();
        k_sleep(K_MSEC(200));
    }

    return 0;
}
