#include "Arduino.h"
#include <MotorCtrl.hpp>
#include <TimeSync.hpp>
#include <ESPNOWCOMM.hpp>

using std::array;

#define SERVO_LEDC_CHANNEL 0
#define SERVO_PIN 12

#define MOTOR_TX_PIN 15
#define MOTOR_RX_PIN 13

#define LED_PIN 9

// servo values
constexpr uint32_t no_trig_val = 457;
constexpr uint32_t trig_val = 435;
constexpr uint32_t full_val = 300;

// ESP-NOW settings
array<uint8_t, 6> station_MAC = {0xF4, 0x12, 0xFA, 0x88, 0x13, 0xD0}; // This is for ESP32-S3 dev board
array<uint8_t, 6> launcher_mac = {0x64, 0xB7, 0x08, 0x9D, 0x69, 0xCC};

constexpr int WiFi_channel = 14;

// global shared variable to indicate whether the launch is intialized or canceled
bool launch_inited = false;
bool launch_canceled = false;
// synced launch time
int64_t synced_launch_time = 0;

// terminal velocity in 0.01dps/LSB (might be slightly inaccurate)
int32_t velocity = 720000;

/**
 * @brief launch the drone task
 */
void Launch(void *pvParameters)
{
    // prevent launching too frequently
    static int64_t last_local_launch_time = 0;
    constexpr int64_t min_launch_time_gap = 10000000;
    if (esp_timer_get_time() - last_local_launch_time < min_launch_time_gap)
    {
        launch_inited = false;
        vTaskDelete(NULL);
    }

    Motor::Resume();
    Motor::Set_velocity(velocity);

    // cancel launch if signaled
    for (int i = 0; i < 40; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (launch_canceled)
        {
            Motor::Set_velocity(0);
            Motor::Stop();
            launch_inited = false;
            vTaskDelete(NULL);
        }
    }

    while (true)
    {
        // get current angle in degrees
        Motor::Read_motor_state();
        float pos = float(Motor::motor_position) / 65536.0F * 360.0F;
        if (345.0F > pos && pos > 335.0F)
        {
            synced_launch_time = TimeSync::Get_synced_time();
            last_local_launch_time = esp_timer_get_time();
            break;
        }
    }

    // trigger servo to launch the drone
    ledcWrite(SERVO_LEDC_CHANNEL, full_val);
    delay(25);

    ledcWrite(SERVO_LEDC_CHANNEL, trig_val);
    delay(100);

    ledcWrite(SERVO_LEDC_CHANNEL, no_trig_val);

    // stop motor after launch
    Motor::Set_velocity(0);
    Motor::Stop();

    launch_inited = false;
    vTaskDelete(NULL);
}

void Trigger_Launch_Task(void *pvParameters)
{
    auto data_queue = TimeSync::Get_data_queue_handle();

    while (true)
    {
        // receive data
        uint8_t temp_data[TimeSync::max_data_len] = {0};
        xQueueReceive(data_queue, temp_data, portMAX_DELAY);
        uint8_t temp_data_len = temp_data[0];

        if (temp_data_len == 4 && TimeSync::SyncedQ() && !launch_inited)
        {
            velocity=*(int32_t*)(temp_data+1);
            launch_canceled = false;
            launch_inited = true;
            xTaskCreate(Launch, "Launch", 10000, nullptr, 10, NULL);
        }
        // cancel the launch if 23 is received
        else if (temp_data_len == 1 && temp_data[1] == 23 && launch_inited)
        {
            launch_inited = false;
            launch_canceled = true;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Program Start!");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // servo init and test
    ledcSetup(SERVO_LEDC_CHANNEL, 300, 10);
    ledcAttachPin(SERVO_PIN, SERVO_LEDC_CHANNEL);

    ledcWrite(SERVO_LEDC_CHANNEL, no_trig_val);
    delay(500);
    ledcWrite(SERVO_LEDC_CHANNEL, trig_val);
    delay(500);
    ledcWrite(SERVO_LEDC_CHANNEL, no_trig_val);
    delay(500);

    // init motor
    Motor::Init(MOTOR_TX_PIN, MOTOR_RX_PIN);
    Motor::Resume();
    Motor::Set_PID(30, 30, 10, 6, 40, 30);

    Motor::Set_incremental_position_2(2000, 6000);
    delay(500);
    Motor::Set_incremental_position_2(-2000, 6000);
    delay(500);
    Motor::Stop();

    // initialize ESP-NOW
    if (ESPNOWCOMM::Init(WiFi_channel) != 0)
    {
        Serial.println("ESP NOW init failure!");
    }

    // register peers
    ESPNOWCOMM::Register_peer(station_MAC);

    TimeSync::Init();
    ESPNOWCOMM::Set_receive_callback(TimeSync::Receive_Callback);

    xTaskCreatePinnedToCore(Trigger_Launch_Task, "Trigger_Launch_Task", 10000, nullptr, 11, NULL, 0);

    Serial.println("Init End!");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(200));
    if (TimeSync::SyncedQ())
    {
        digitalWrite(LED_PIN, HIGH);
    }
    else
    {
        digitalWrite(LED_PIN, LOW);
    }

    // send, send and send
    if (synced_launch_time != -1 && synced_launch_time != 0)
    {
        // send once here, send many more later...
        // convert launch time to byte array and send
        uint8_t synced_launch_time_bytes[8];
        memcpy(synced_launch_time_bytes, &synced_launch_time, 8);
        ESPNOWCOMM::Send(0, synced_launch_time_bytes, 8);
    }
}
