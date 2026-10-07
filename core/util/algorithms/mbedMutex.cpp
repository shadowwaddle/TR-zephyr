/*
This is here so that porting the old Mbed Mutex to Zephyr is easier. Could probably be cleaned up later - Dil
*/
#pragma once 
// Ignore this warning, we actually do need this here because I defined my class in a cpp lmao
#include <zephyr/kernel.h>


class Mutex {
public:
    Mutex() { k_mutex_init(&m_mutex); }
    void lock() { k_mutex_lock(&m_mutex, K_FOREVER); }
    void unlock() { k_mutex_unlock(&m_mutex); }
    bool trylock() { return k_mutex_lock(&m_mutex, K_NO_WAIT) == 0; }

private:
    struct k_mutex m_mutex;
};