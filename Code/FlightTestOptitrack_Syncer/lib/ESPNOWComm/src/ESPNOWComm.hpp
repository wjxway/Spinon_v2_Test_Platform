/**
 * @file ESPNOWComm.hpp
 * @brief a wrapped up package to do espnow communication
 */
#ifndef ESPNOWCOMM_HPP__
#define ESPNOWCOMM_HPP__

#include "Arduino.h"
#include <array>

namespace ESPNOWCOMM
{
    /**
     * @brief initialize ESPNOW to communicate with **ONE** other peer
     *
     * @param MAC MAC address of peer
     * @param channel WiFi channel to communicate on, make sure this is
     * consistent across devices.
     * @return int if 0 successful, if 1 or 2 not successful
     */
    int Init(const int channel);

    /**
     * @brief Register a peer to the peer list
     *
     * @param MAC MAC address of the peer
     * @return int index of the peer in the list, -1 if failed.
     *
     * @note can also be used to check the position of the peer in the list
     */
    int Register_peer(const std::array<uint8_t, 6> MAC);

    /**
     * @brief send data to peer
     *
     * @param target negative for broadcast, otherwise index of peer in the list
     * @param data data to send
     * @param len length of data
     * @return esp_err_t
     */
    esp_err_t Send(const int target, const uint8_t *const data, const size_t len);

    /**
     * @brief set certain function as receive callback
     * @param callback a function like void Receive_Callback(const uint8_t *mac_addr, const uint8_t *data, int data_len)
     *
     * @return esp_err_t ESP_OK means succeed.
     *
     * @note this is just a simple wrapper around esp_now_register_recv_cb()
     * @warning this callback is inside WiFi's ISR, so you shouldn't do anything time consuming here!
     */
    esp_err_t Set_receive_callback(void (*callback)(const uint8_t *mac_addr, const uint8_t *data, int data_len));
}

#endif