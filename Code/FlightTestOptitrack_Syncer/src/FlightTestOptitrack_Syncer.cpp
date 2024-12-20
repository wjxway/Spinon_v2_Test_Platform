/**
 * @file FlightTestOptitrack.cpp
 * @brief main file
 */
#include <cstring>
#include <DebugDefs.hpp>
#include <ESPNOWComm.hpp>
#include <FastIO.hpp>

using std::array;

#define TRIG_PIN 15
#define OUT_PIN 13

// ESP-NOW settings
// array<uint8_t, 6> station_MAC = {0x64, 0xB7, 0x08, 0x9D, 0x72, 0x54}; // This is for ESP32 dev board
array<uint8_t, 6> station_MAC = {0xF4, 0x12, 0xFA, 0x88, 0x13, 0xD0}; // This is for ESP32-S3 dev board
// array<uint8_t, 6> spinon_MAC = {0x64, 0xE8, 0x33, 0x72, 0x02, 0x94}; // Spinon 21
array<uint8_t, 6> spinon_MAC = {0x64, 0xE8, 0x33, 0x72, 0x02, 0xF4}; // Spinon 22
array<uint8_t, 6> launcher_MAC = {0x64, 0xB7, 0x08, 0x9D, 0x69, 0xCC};

constexpr int WiFi_channel = 14;

// frame rate of optitrack
constexpr int optitrack_frame_rate = 360;
// each frame's time in us
constexpr int optitrack_frame_time = 1000000 / optitrack_frame_rate;

// current frame number
// -1 means not started
int64_t curr_frame = -1;
// when is frame 0 in esp_timer_get_time() (back traced time)
int64_t curr_frame_time = 0;

// maximum length of data in bytes, please leave some more margin here
// 1. note that the maximum length of ESP-NOW data is 257 bytes.
// 2. note that the data should be 8 bytes less than this value due to the
// addition of sync information.
// 3. note that we do not validate for length in the code, so make sure your data is 10 bytes less than this value just for sure.
constexpr size_t max_data_len = 100;

// when syncer received any message, it converts the message to raw char stream
// and send to computer.
// the first byte is the index of the peer, the rest is the data.
void Receive_Callback(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    uint8_t datatemp[max_data_len] = {0};

    // get the index of the peer
    std::array<uint8_t, 6> tempmac;
    memcpy(tempmac.data(), mac_addr, 6);
    datatemp[0] = (uint8_t)ESPNOWCOMM::Register_peer(tempmac);

    // copy the data
    memcpy(datatemp + 1, data, data_len);

    Serial.write(datatemp, data_len + 1);
    Serial.flush(true);
}

// synchronize with Optitrack eSync
void Sync_ISR()
{
    int64_t t_now = esp_timer_get_time();

    // exit if this is a false trigger
    constexpr int64_t trigger_time_tol = 100;
    if (curr_frame_time != 0 && abs(t_now - curr_frame_time - optitrack_frame_time) >= trigger_time_tol)
    {
        return;
    }

    // update frame and frame time
    curr_frame++;
    curr_frame_time = t_now;

    // toggle out pin for test
    if (curr_frame % 100 == 0)
    {
        setbit(OUT_PIN);
    }
    else if (curr_frame % 100 == 50)
    {
        clrbit(OUT_PIN);
    }
}

// if no triggering within a period of time, we reset the trigger value to 0.
void Trigger_Reset_Task(void *pvParameters)
{
    constexpr int keep_alive_time = 1000; // how many ms per update

    int64_t last_checked_frame = -1;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(keep_alive_time));

        // reset if there's no incoming valid pulse in a while
        if (curr_frame == last_checked_frame && curr_frame != -1)
        {
            curr_frame = -1;
            curr_frame_time = 0;
        }

        last_checked_frame = curr_frame;
    }
}

// if any message has been sent in the last period
int msg_sent = 0;
// if no message has been sent within a period of time
// we send an empty message to keep time sync
void Timesync_Task(void *pvParameters)
{
    // make sure there's a transmission per this time in ms
    constexpr int timesync_max_period = 15;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(timesync_max_period));

        int64_t this_frame;
        int64_t t_since_start;
        do
        {
            this_frame = curr_frame;

            // frame number & offset from frame number, unit in us
            // or you can consider this as time since 0th frame
            t_since_start = 1000000 * curr_frame / optitrack_frame_rate + (esp_timer_get_time() - curr_frame_time);
        } while (curr_frame != this_frame);

        // send an empty message if no message has been sent for a while
        if (!msg_sent && this_frame >= 0)
        {
            ESPNOWCOMM::Send(-1, (uint8_t *)(&t_since_start), sizeof(t_since_start));
        }

        msg_sent = 0;
    }
}

void setup()
{
    Serial.begin(2000000);
    DEBUG_C(Serial.println("------ Init start! ------"));

    pinMode(TRIG_PIN, INPUT);
    attachInterrupt(TRIG_PIN, Sync_ISR, RISING);

    pinMode(OUT_PIN, OUTPUT);
    setbit(OUT_PIN);

    // initialize ESP-NOW
    if (ESPNOWCOMM::Init(WiFi_channel) != 0)
    {
        DEBUG_C(Serial.println("ESP NOW init failure!"));
    }

    // // obtain local mac address, please remember to change the Serial's baud rate to properly read it.
    // uint8_t mac[6];
    // esp_read_mac(mac, ESP_MAC_WIFI_STA);
    // Serial.printf("Local MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // delay(10000);

    // register peers
    ESPNOWCOMM::Register_peer(spinon_MAC);
    ESPNOWCOMM::Register_peer(launcher_MAC);

    ESPNOWCOMM::Set_receive_callback(Receive_Callback);

    xTaskCreate(Trigger_Reset_Task, "Trigger_Reset_Task", 5000, nullptr, 11, NULL);
    xTaskCreate(Timesync_Task, "Timesync_Task", 5000, nullptr, 11, NULL);

    DEBUG_C(Serial.println("------ Init finished! ------"));
}

// delete loop() immediately
void loop()
{
    if (Serial.available())
    {
        // delay for a little bit and let data finish
        delayMicroseconds(10);

        uint8_t data[max_data_len] = {0};
        int data_len = Serial.read(data, max_data_len - sizeof(int64_t));

        int64_t this_frame;
        int64_t t_since_start;
        do
        {
            this_frame = curr_frame;

            // frame number & offset from frame number, unit in us
            // or you can consider this as time since 0th frame
            t_since_start = 1000000 * curr_frame / optitrack_frame_rate + (esp_timer_get_time() - curr_frame_time);
        } while (curr_frame != this_frame);

        // send only if recording has started
        if (this_frame >= 0)
        {
            memcpy(data + data_len, &t_since_start, sizeof(t_since_start));

            // first byte is the index of the peer, negative for broadcast
            ESPNOWCOMM::Send((int8_t)data[0], data + 1, data_len + sizeof(t_since_start) - 1);
            msg_sent = 1;
        }

        // clear up Serial after a read
        Serial.flush(false);
    }

    // // Test task
    // vTaskDelay(pdMS_TO_TICKS(5));
    //
    // int64_t t_now = esp_timer_get_time();
    // int64_t this_frame;
    // int64_t t_since_start;
    // do
    // {
    //     this_frame = curr_frame;
    //
    //     // frame number & offset from frame number, unit in us
    //     // or you can consider this as time since 0th frame
    //     t_since_start = 1000000 * curr_frame / optitrack_frame_rate + (t_now - curr_frame_time);
    //
    // } while (curr_frame != this_frame);
    //
    // // send only if recording has started
    // if (this_frame >= 0)
    // {
    //     uint8_t data[20] = {'S', 'E', 'R', 'V', 'E', 'R', '\n', '\0'};
    //     int data_len = 7;
    //     memcpy(data + data_len, &t_since_start, sizeof(t_since_start));
    //     ESPNOWCOMM::Send(data, data_len + sizeof(t_since_start));
    // }
}
