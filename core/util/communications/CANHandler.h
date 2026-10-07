///////////////////////////////////////////////////////////////////////
//                        ____               __       __  __  _ _____                   ____               ____     ___   __   _ ___    __  _          
// _______ ___ _____ ___ / __/__  ___ ____ _/ /  ___ / /_/ /_(_) ___/__  __ _  ___ ___ / __ \___  _______ /  _/__  / _ | / /  (_) _/__ / /_(_)_ _  ___ 
/// __/ _ `/ // (_-</ -_)\ \/ _ \/ _ `/ _ `/ _ \/ -_) __/ __/ / /__/ _ \/  ' \/ -_|_-</ /_/ / _ \/ __/ -_)/ // _ \/ __ |/ /__/ / _/ -_) __/ /  ' \/ -_)
//\__/\_,_/\_,_/___/\__/___/ .__/\_,_/\_, /_//_/\__/\__/\__/_/\___/\___/_/_/_/\__/___/\____/_//_/\__/\__/___/_//_/_/ |_/____/_/_/ \__/\__/_/_/_/_/\__/ 
//                        /_/        /___/                                                                                                             
//
///////////////////////////////////////////////////////////////////////
// Yes the PKnessness Graffeti is indeed necessary
#pragma once
// #include "CANMsg.h"
#include <cerrno>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <vector>
#include <functional>

struct ExactCallback {
    uint32_t id;
    std::function<void(const can_frame*)> func;
};

struct RangeCallback {
    uint32_t start_id;
    uint32_t end_id;
    std::function<void(const can_frame*)> func;
};

class CANHandler{
    
public:

    ~CANHandler() {
    if (filter_id >= 0) {
        can_remove_rx_filter(dev_, filter_id);
    }
}
    enum CANBus{CANBUS_1, CANBUS_2, NOBUS}; // Somewhat vestigal, makes porting mbedOS code a little easier - Dil 

    explicit CANHandler(const struct device *can_dev)
        : dev_(can_dev)
    {
        k_msgq_init(&rx_msgq_, rx_msgq_buf_, sizeof(struct can_frame), RX_QUEUE_DEPTH);
    }
    
    int init() {
        if (!device_is_ready(dev_)) {
            return -ENODEV;
        }

        const struct can_filter catch_all = {.id =0, .mask = 0, .flags = 0};
        filter_id = can_add_rx_filter_msgq(dev_, &rx_msgq_, &catch_all);
        if (filter_id < 0) {
            return filter_id;
        }
        return can_start(dev_);
    }

    void registerCallback(uint32_t id, std::function<void(const can_frame*)> func) {
        exact_.push_back({id, std::move(func)});
    }

    void registerCallback(uint32_t start_id, uint32_t end_id, std::function<void(const can_frame*)> func) {
        range_.push_back({start_id, end_id, std::move(func)});
    }

    void readAllCan() {
        struct can_frame raw;
        // CANMsg rxMsg;
        struct can_frame rxMsg;

        while (k_msgq_get(&rx_msgq_, &raw, K_NO_WAIT) == 0) {
            // rxMsg.msg = raw;
            rxMsg = raw;

            for (auto &cb : exact_) {
                // if (rxMsg.msg.id == cb.id) {
                if (rxMsg.id == cb.id) {
                    cb.func(&rxMsg);
                }
            }
            for (auto &cb : range_) {
                // if (rxMsg.msg.id >= cb.start_id && rxMsg.msg.id <= cb.end_id) {
                if (rxMsg.id >= cb.start_id && rxMsg.id <= cb.end_id) {
                    cb.func(&rxMsg);
                }
            }
        }
    }

    bool rawSend(uint32_t id, const uint8_t bytes[], uint8_t length = 8) {
        // CANMsg tx(id, bytes, length);
        struct can_frame tx = {
            .id = id,
            .dlc = length,
            .flags = 0,
        };
        memcpy(tx.data,bytes,length);
        int ret = can_send(dev_, &tx, K_MSEC(100), nullptr, nullptr);
        return ret == 0;
    }

    enum can_state getState() const {
    enum can_state state;
    struct can_bus_err_cnt err_cnt;
    can_get_state(dev_, &state, &err_cnt);
    return state;
}


private:
    static constexpr int RX_QUEUE_DEPTH = 128;

    const struct device *dev_;
    int filter_id = -1;

    struct k_msgq rx_msgq_;
    char rx_msgq_buf_[RX_QUEUE_DEPTH * sizeof(struct can_frame)] __aligned(4);

    std::vector<ExactCallback> exact_;
    std::vector<RangeCallback> range_;
};