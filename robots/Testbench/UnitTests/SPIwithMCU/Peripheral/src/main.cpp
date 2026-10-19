#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));

// setting device config as slave
static struct spi_config spi_cfg = {
    .frequency = 1000000,
    .operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave = 0,
};

// spi helper transfer and recieves 1 byte

uint8_t transferSPI(uint8_t tx) {
    uint8_t rx = 0;
    struct spi_buf tx_buf = { .buf = &tx, .len = 1 };
    struct spi_buf rx_buf = { .buf = &rx, .len = 1 };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };
    int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    if (ret < 0) {
        printk("SPI transfer failed: %d\n", ret);
        return 0;
    }
    return rx;
}

void periodic() {
    uint8_t received = transferSPI(0xAA);
    printk("Received: 0x%02X\n", received);
}

int main(void)
{
    if (!device_is_ready(spi_dev)) {
        printk("SPI1 not ready!\n");
        return -1;
    }

    printk("SPI1 ready!\n");

    while (true) {
        periodic();
    }

    return 0;
}