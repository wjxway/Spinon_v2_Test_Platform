/**
 * @file TimeSync.hpp
 * @brief time synchronization through ESP-NOW
 */
#ifndef TIMESYNC_HPP__
#define TIMESYNC_HPP__

#include "Arduino.h"

namespace TimeSync
{
    // data queue
    constexpr size_t queue_max_len = 100;
    constexpr size_t max_data_len = 100;

    /**
     * @brief Initialize data structures
     */
    void Init();

    /**
     * @brief Callback to add to ESP-NOW
     * 
     * @param mac_addr
     * @param data 
     * @param data_len 
     */
    void Receive_Callback(const uint8_t *mac_addr, const uint8_t *data, int data_len);

    /**
     * @brief get the synchronized time
     * 
     * @return int64_t time in us, if is 0 then not synced
     */
    int64_t Get_synced_time();

    /**
     * @brief get the time offset, esp_timer_get_time() - offset -> synced_time
     * 
     * @return int64_t time in us, if is 0 then not synced.
     */
    int64_t Get_synced_time_offset();

    /**
     * @brief return the handle to data queue
     * @return QueueHandle_t 
     * 
     * @note elements in data_queue is uint8_t arrays:
     *      1. their first element is the length of data
     *      2. their subsequent bytes are real data up to the length
     */
    QueueHandle_t Get_data_queue_handle();

    /**
     * @brief if synchronized
     * 
     * @return true 
     * @return false 
     */
    bool SyncedQ();
} // namespace TimeSync

#endif