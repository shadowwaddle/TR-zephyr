#include "DMAUart.h"
#include <cstdio>

#define LOG_MISSED_PACKETS 1

DMAUart::DMAUart(const struct device *dev) : uart_dev(dev) {
    ring_buf_init(&m_rx_rb, sizeof(m_rx_rb_backing), m_rx_rb_backing);
    k_sem_init(&m_rx_sem, 0, 1);

    if (!device_is_ready(uart_dev)) {
        printf("[ERROR] DMAUart: device not ready\n");
        return;
    }

    int ret = uart_callback_set(uart_dev, &DMAUart::uart_cb_trampoline, this);
    if (ret != 0) {
        printf("[ERROR] DMAUart: uart_callback_set failed: %d\n", ret);
        return;
    }

    m_next_dma_buf_idx = 1;
    start_rx(m_dma_buf[0], DMA_RX_BUF_SIZE);
}

void DMAUart::start_rx(uint8_t *buf, size_t len) {
    int ret = uart_rx_enable(uart_dev, buf, len, DMA_RX_TIMEOUT_US);
    if (ret != 0) {
        printf("[ERROR] DMAUart: start_rx failed, GG");
        return;
    }
}

void DMAUart::uart_cb(struct uart_event *evt) {
    switch (evt -> type) {
    
    case UART_RX_RDY: {
        // New data landed in the buffer currently owned by the DMA controller.
        // Copy it into our own ring buffer so read() has a stable place to
        // pull from, independent of which of the two DMA buffers is "hot".
        uint32_t put = ring_buf_put(&m_rx_rb,
                                     evt->data.rx.buf + evt->data.rx.offset,
                                     evt->data.rx.len);
        #if LOG_MISSED_PACKETS == 1
        atomic_add(&m_total_rx, evt->data.rx.len);
        if (put < evt->data.rx.len) atomic_add(&m_dropped, evt->data.rx.len - put);
        break;
        #endif
    }

    case UART_RX_BUF_REQUEST: {
        // Driver wants the next landing buffer queued up now, before the
        // current one fills, so the DMA transfer never has a gap.
        uint8_t *next = m_dma_buf[m_next_dma_buf_idx];
        m_next_dma_buf_idx ^= 1;
        uart_rx_buf_rsp(uart_dev, next, DMA_RX_BUF_SIZE);
        break;
    }

    case UART_RX_BUF_RELEASED:
        // Buffer we handed back earlier is free again; nothing to do since
        // we own both buffers statically for the object's whole lifetime.
        break;

    case UART_RX_DISABLED:
        // Some STM32 families disable RX on certain error conditions
        // (framing/overrun/parity) and need an explicit restart.
        m_next_dma_buf_idx = 1;
        start_rx(m_dma_buf[0], DMA_RX_BUF_SIZE);
        m_next_dma_buf_idx = 1;
        break;

    case UART_RX_STOPPED:
        printf("[WARNING] DMAUart: rx stopped, reason %d\n", evt->data.rx_stop.reason);
        break;

    default:
        break;
    }

}


ssize_t DMAUart::read(void *buffer, size_t length) {
    return static_cast<ssize_t>(ring_buf_get(&m_rx_rb, static_cast<uint8_t *>(buffer), length));
}

bool DMAUart::readable() {
    return !ring_buf_is_empty(&m_rx_rb);
}

void DMAUart::printMissedPackets() {
    #if LOG_MISSED_PACKETS == 1
    int total = atomic_get(&m_total_rx);
    int dropped = atomic_get(&m_dropped);
    if (total > 0) {
        printf("[INFO] DMAUart: total rx %d, dropped %d, loss %.2f%%\n", total, dropped, (float)dropped / total * 100.0f);
        atomic_set(&m_total_rx, 0);
        atomic_set(&m_dropped, 0);
    }
    #endif
}