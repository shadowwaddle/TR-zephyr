#include "util/communications/jetson/Jetson.h"
#include <util/algorithms/general_functions.h>

#define JETSON_READ_STACK_SIZE 4096
#define JETSON_WRITE_STACK_SIZE 2048
#define JETSON_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(jetson_read_stack, JETSON_READ_STACK_SIZE);
K_THREAD_STACK_DEFINE(jetson_write_stack, JETSON_WRITE_STACK_SIZE);

Jetson::Jetson(const struct device *UARTJetson)
    : jetsonSerial(UARTJetson, nullptr), jetsonSPI(nullptr), jetsonMode(UART) {
    write_packets_.push_back(std::make_unique<RefWritePacket>());
    write_packets_.push_back(std::make_unique<RobotStateWritePacket>());
    write_packets_.push_back(std::make_unique<EmbeddedUserInputWritePacket>());

    read_packets_.push_back(std::make_unique<TurretPacket>());
    read_packets_.push_back(std::make_unique<ChassisReadPacket>());

    // bcJetson->set_blocking(true);

    // this->write_thread_.start(callback(this, &Jetson::writeThread));

    k_thread_create(&m_write_tdata, jetson_write_stack, JETSON_WRITE_STACK_SIZE,
    writeThreadEntry, this, NULL, NULL,
    JETSON_THREAD_PRIORITY, 0, K_NO_WAIT);

    
    // this->read_thread_.start(callback(this, &Jetson::readThread));
    k_thread_create(&m_read_tdata, jetson_read_stack, JETSON_READ_STACK_SIZE,
    readThreadEntry, this, NULL, NULL,
    JETSON_THREAD_PRIORITY, 0, K_NO_WAIT);
    
}

// TODO need to properly add SPI support
// Jetson::Jetson(SPISlave &SPIJetson)
//     : bcJetson(nullptr), spiJetson(&SPIJetson) {}

Jetson::ReadState Jetson::read() {
    this->mutex_read_.lock();
    auto to_return = this->read_state_;
    this->mutex_read_.unlock();

    return to_return;
}

void Jetson::write(Jetson::WriteState &to_write) {
    this->mutex_write_.lock();
    this->write_state_ = to_write;
    this->mutex_write_.unlock();
}

/**
 * Performs a Longitudinal Redundancy Check
 * @param data the data to compute the checksum on
 * @param length the length of data
 */
uint8_t Jetson::calculateLRC(const char *data, size_t length) {
    unsigned char lrc = 0;
    for (size_t i = 0; i < length; ++i) {
        lrc += data[i];
        lrc &= 0xff;
    }
    lrc = ((lrc ^ 0xff) + 1) & 0xff;
    return lrc;
}

// TODO implement SPI here
int Jetson::readIO(char *buff, int buff_size) {
    return jetsonSerial.read(buff, buff_size);
}

// TODO implement SPI here
int Jetson::readIOReadable() { return jetsonSerial.readable(); }

void Jetson::readThread() {
    int constexpr BUFF_SIZE = 512;
    char buff[BUFF_SIZE];
    int buff_tail = 0;

    while (1) {
        if (readIOReadable()) {
            // because we check readable first this should not block waiting for
            // data
            int bytes_read = readIO(&buff[buff_tail], BUFF_SIZE - buff_tail);
            if (bytes_read < 0) {
                continue;
            }

            buff_tail += bytes_read;

            // search for packets left to right.
            int buff_head = 0;
            int start_of_partial_data = 0;
            while (buff_head < buff_tail) {
                bool found_packet = false;
                int bytes_consumed = 0;
                for (auto &packet : read_packets_) {
                    mutex_read_.lock();
                    // clang-format off
                    bytes_consumed = packet->parse_buff(&buff[buff_head], buff_tail - buff_head, read_state_);
                    read_state_.stamp_us = now_us();
                    // clang-format on
                    mutex_read_.unlock();

                    // found a match
                    if (bytes_consumed > 0) {
                        found_packet = true;
                        // dont look for anymore packets at the current idx
                        break;
                    }
                }

                if (found_packet) {
                    buff_head += bytes_consumed;
                    start_of_partial_data = buff_head;
                } else {
                    buff_head++;
                }
            }

            // if a packet is found and there is data to the right of it, copy
            // that data to the start of the buffer so it can be potentially
            // matched to a packet as more data arrives
            if (start_of_partial_data != 0) {
                memmove(&buff[0], &buff[start_of_partial_data],
                             buff_tail - start_of_partial_data);

                buff_tail = buff_tail - start_of_partial_data;
            }

            // dumb safety check a circular buffer is probably better
            if (buff_tail > 400) {
                buff_tail = 0;
                printf("[ERROR] jetson read buffer resetting");
            }
        }
        // ThisThread::yield();
        k_yield();
    }
}

// TODO add spi support here
int Jetson::writeIO(char *buff, int write_size) {
    return jetsonSerial.write(buff, write_size);
}

void Jetson::writeThread() {
    // unsigned long curr_time = now_us();
    // unsigned long prev_time = now_us();
    while (1) {
        char buff[256];
        int buff_pos = 0;

        // curr_time = now_us();

        // if ((curr_time - prev_time) / 1000 >= WRITE_THREAD_LOOP_DT_MS) {
        //     prev_time = curr_time;

            for (auto &packet : write_packets_) {
                mutex_write_.lock();
                int bytes_wrote =
                    packet->write_data_to_buff(write_state_, buff + buff_pos, 256);
                mutex_write_.unlock();

                if (bytes_wrote < 0) {
                    continue;
                }

                buff_pos += bytes_wrote;
            }

            writeIO(buff, buff_pos);
        // }
        k_sleep(K_MSEC(WRITE_THREAD_LOOP_DT_MS));
    }
}

void Jetson::readThreadEntry(void *p1, void *p2, void *p3) {
    static_cast<Jetson *>(p1)->readThread();
}

void Jetson::writeThreadEntry(void *p1, void *p2, void *p3) {
    static_cast<Jetson *>(p1)->writeThread();
}