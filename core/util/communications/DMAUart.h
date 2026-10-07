#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/sys/ring_buffer.h>
#include <cstdint>


// ADJUST THESE VALUES AS NECESSARY
#define DMA_RX_BUF_SIZE   256   // size of each of the two DMA landing buffers
#define DMA_RX_RING_SIZE  1024   // ring buffer bytes get copied into for consumption by read()
#define DMA_RX_TIMEOUT_US 1000  // inactivity gap that flushes a partial UART_RX_RDY event -- tune to your frame spacing

/*
This class is meant to be as similar to mbedSerial/serialbase as possible 
*/
class DMAUart {
    public: 
        explicit DMAUart(const struct device *dev);

        ssize_t read(void *buffer, size_t length);
        bool readable();

        void printMissedPackets();

    private:

        /**
        @brief Standard static trampoline function that bounces us to the specific instance of DMAUart that we care about. We need this because of the way zephyr
        implements UART.  
         */
        static void uart_cb_trampoline(const struct device *dev, struct uart_event *evt, void *user_data) {
            static_cast<DMAUart *>(user_data)->uart_cb(evt);
        }
        void uart_cb(struct uart_event *evt);
        void start_rx(uint8_t *buf, size_t len);
        
        const struct device *uart_dev;

        // We have two buffers here so we can immediately swap to the other while we drain the first
        // (DMA loads them, CPU has to then read them and place them into the ring buffer to work with them)
        uint8_t m_dma_buf[2][DMA_RX_BUF_SIZE];
        uint8_t m_next_dma_buf_idx = 0;

        // Ring buffers are just FIFO buffers that the CPU does its thing with 

        // ring_buf struct is meant for book keeping
        struct ring_buf m_rx_rb;

        // Actual storage 
        uint8_t m_rx_rb_backing[DMA_RX_RING_SIZE];

        struct k_sem m_rx_sem; // This is currently unused, but we might want to implement this later. 

        atomic_t m_total_rx = 0;
        atomic_t m_dropped = 0;
};