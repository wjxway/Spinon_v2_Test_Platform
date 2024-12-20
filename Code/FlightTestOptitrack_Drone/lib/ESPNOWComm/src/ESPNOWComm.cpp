#include "ESPNOWComm.hpp"
#include <esp_wifi.h>
#include <esp_now.h>

#include <vector>
using namespace std;

namespace ESPNOWCOMM
{
    namespace
    {
        vector<array<uint8_t, 6>> peer_MAC_list = {};
    }

    int Init(const int channel)
    {
        esp_netif_init();
        esp_event_loop_create_default();
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_start();
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_12M);

        if (esp_now_init() != ESP_OK)
        {
            return 1;
        }

        return 0;
    }

    int Register_peer(const array<uint8_t, 6> MAC)
    {
        // check if MAC is already in list
        for (int i = 0; i < peer_MAC_list.size(); i++)
        {
            if (peer_MAC_list[i] == MAC)
            {
                return i;
            }
        }

        // if not, add to list
        peer_MAC_list.push_back(MAC);

        // add peer
        esp_now_peer_info_t peerInfo{};

        memcpy(peerInfo.peer_addr, MAC.data(), 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        // Add peer
        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            return -1;
        }

        return peer_MAC_list.size() - 1;
    }

    /**
     * @brief send data to peer
     *
     * @param target negative for broadcast, otherwise index of peer in the list
     * @param data data to send
     * @param len length of data
     * @return esp_err_t
     */
    esp_err_t Send(const int target, const uint8_t *const data, const size_t len)
    {
        // if target is negative, broadcast
        if (target < 0)
        {
            return esp_now_send(NULL, data, len);
        }

        // return error if target is out of range
        if (target >= peer_MAC_list.size())
        {
            return ESP_ERR_ESPNOW_NOT_INIT;
        }

        // send to target
        return esp_now_send(peer_MAC_list[target].data(), data, len);
    }

    esp_err_t Set_receive_callback(void (*callback)(const uint8_t *mac_addr, const uint8_t *data, int data_len))
    {
        return esp_now_register_recv_cb(callback);
    }
}