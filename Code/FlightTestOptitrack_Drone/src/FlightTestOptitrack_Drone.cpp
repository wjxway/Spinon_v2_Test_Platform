/**
 * @file FlightTestOptitrack.cpp
 * @brief main file
 */
#include <cstring>
#include <MotorCtrl.hpp>
#include <DebugDefs.hpp>
#include <ESPNOWComm.hpp>
#include <TimeSync.hpp>
#include <INA.h>
#include "Tasks.hpp"
#include "HardwareDefs.hpp"

using std::array;

// Power monitoring setup
INA_Class INA; // INA class instance for power monitoring

// // temp
// hw_timer_t *temp_timer;
// void Send_pulse_ISR();
// void Send_pulse_task(void *pvParameters);

void setup()
{
    // RGB pins
    pinMode(LED_B, OUTPUT);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);

    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    vTaskDelay(500);
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    vTaskDelay(500);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    vTaskDelay(500);
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);

    Serial.begin(115200);
    Serial.printf("------ Init start on core %d! ------\n", xPortGetCoreID());

    Motor::Init();

    // Initialize INA power monitoring
    Serial.println("Initializing INA power monitoring...");
    uint8_t devicesFound = INA.begin(MAXIMUM_AMPS, SHUNT_MICRO_OHM);
    if (devicesFound == 0)
    {
        Serial.println("Warning: No INA device found!");
    }
    else
    {
        Serial.printf("Found %d INA device(s)\n", devicesFound);
        // Configure INA settings for continuous monitoring
        INA.setBusConversion(8500);            // Maximum conversion time 8.244ms
        INA.setShuntConversion(8500);          // Maximum conversion time 8.244ms
        INA.setAveraging(16);                  // Average readings for better accuracy
        INA.setMode(INA_MODE_CONTINUOUS_BOTH); // Continuously measure bus and shunt
        Serial.println("INA power monitoring initialized successfully");
    }

    if (ESPNOWCOMM::Init(WiFi_channel) != 0)
    {
        printf("ESP NOW init failure!\n");
    }
    ESPNOWCOMM::Register_peer(station_MAC);

    // // obtain local mac address, please remember to change the Serial's baud rate to properly read it.
    // uint8_t mac[6];
    // esp_read_mac(mac, ESP_MAC_WIFI_STA);
    // Serial.printf("Local MAC: {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // delay(10000);

    TimeSync::Init();
    ESPNOWCOMM::Set_receive_callback(TimeSync::Receive_Callback);

    // // for sync test, note that it cannot coexist with Motor!
    // pinMode(34, OUTPUT);
    // digitalWrite(34, LOW);
    // temp_timer = timerBegin(0, 80, true);
    // timerAttachInterrupt(temp_timer, Send_pulse_ISR, true);
    // xTaskCreate(Send_pulse_task, "Send_pulse_task", 5000, nullptr, 11, NULL);

    // xTaskCreate(Print_data_task, "Print_data_task", 5000, nullptr, 11, NULL);

    xTaskCreatePinnedToCore(Set_Throttle_Task, "Set_Throttle_Task", 10000, nullptr, 11, NULL, 1);
    xTaskCreatePinnedToCore(Send_FB_Task, "Send_FB_Task", 10000, nullptr, 5, NULL, 0);
    // xTaskCreatePinnedToCore(PrintTask, "PrintTask", 5000, nullptr, 2, NULL, 1);

    Serial.println("------ Init finished! ------");
}

// delete loop() immediately
void loop()
{
    // vTaskDelay(200);
    // uint8_t data[10] = {'D', 'A', 'T', 'A', '\n', '\0'};
    // ESPNOWCOMM::Send(data, 5);

    vTaskDelay(pdMS_TO_TICKS(200));
    if (TimeSync::SyncedQ())
    {
        digitalWrite(LED_B, LOW);
    }
    else
    {
        digitalWrite(LED_B, HIGH);
    }
}

// // example of reading data
// void Print_data_task(void *pvParameters)
// {
//     while (1)
//     {
//         uint8_t temp_data[max_data_len] = {0};
//         xQueueReceive(data_queue, temp_data, portMAX_DELAY);

//         Serial.printf("%lld : %s", t_offset, (char *)temp_data);
//     }
// }

// // example of how to trigger at specific time
// int output_state = 0;
// void Send_pulse_task(void *pvParameters)
// {
//     while (1)
//     {
//         vTaskDelay(pdMS_TO_TICKS(5));

//         int64_t t_since_start = TimeSync::Get_synced_time();
//         if (t_since_start != 0)
//         {
//             int64_t curr_frame = t_since_start / 2000;
//             if (curr_frame % 50 <= 40)
//             {
//                 output_state = (curr_frame % 100 >= 50);

//                 timerRestart(temp_timer);
//                 timerAlarmWrite(temp_timer, (curr_frame / 50 + 1) * 50 * 2000 - t_since_start, false);
//                 timerAlarmEnable(temp_timer);
//             }
//         }
//     }
// }

// // if no triggering within a period of time, we reset the trigger value to 0.
// void Send_pulse_ISR()
// {
//     digitalWrite(34, output_state);
// }

// // motor speed test code to put in setup()
// auto buf = Motor::Get_motor_speed_buffer();

// for (int th = 500; th <= 2000; th += 250)
// {
//     Serial.printf("--- %d ---\n",th);

//     for (int i = 0; i < 150; i++)
//     {
//         delay(15);
//         Motor::Set_throttle(th);
//     }

//     while (!buf.Empty_Q())
//     {
//         Serial.println(buf.pop().eRPM);
//     }

//     for (int i = 0; i < 100; i++)
//     {
//         delay(15);
//         Motor::Set_throttle(500);
//     }
// }

// Motor::Set_throttle(0);