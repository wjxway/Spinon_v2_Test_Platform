#include "Tasks.hpp"
#include <ESPNOWComm.hpp>
#include <MotorCtrl.hpp>
#include <TimeSync.hpp>

#include <cstring>

constexpr float LED_angle_offset = 0.7712F;

// struct specs
// make sure you properly align the memory
// i.e. 8 bytes at a time

// command for motor
union Command
{
    struct
    {
        int64_t zero_time;
        int32_t period;
        uint32_t t_offset; // when to start switching from throttle 0 -> 1
        uint32_t throttle_0;
        uint32_t throttle_1;
    };
    uint8_t raw[24];
};

union AngleMsg
{
    struct
    {
        int64_t zero_time;
        int32_t period;
        uint32_t throttle;
    };
    uint8_t raw[16];
};

#define LED_R 48

namespace
{
    esp_timer_handle_t tm_handle;

    // avoid concurrency issues
    DRAM_ATTR Command cmd_info_list[2] = {{{0}}, {{0}}};
    // if cmd_info_pos increased then new value is present
    DRAM_ATTR uint32_t cmd_info_pos = 0;
    // last time info is updated
    DRAM_ATTR int64_t last_update_time = 0;
    // if ISR started
    bool ISR_started = false;

    IRAM_ATTR void set_cmd(const Command &val)
    {
        cmd_info_list[1 - cmd_info_pos] = val;
        cmd_info_pos = 1 - cmd_info_pos;
        last_update_time = esp_timer_get_time();
    }

    IRAM_ATTR Command get_cmd()
    {
        return cmd_info_list[cmd_info_pos];
    }

    DRAM_ATTR Command curr_cmd;

    // 0,1,2 for throttle 0, throttle 1, throttle 0
    int DRAM_ATTR output_state = 2;

    // TaskHandle_t waketaskhandle;

    String debug_str = "";

    hw_timer_t *hw_tm;
}

// the skip current phase version
// if no triggering within a period of time, we reset the trigger value to 0.
// IRAM_ATTR void Change_throttle_callback(void *pvParameters)
IRAM_ATTR void Change_throttle_callback()
{
    constexpr int32_t min_phase_time = 400; // minimum time spent on each phase

    int32_t set_motor_throttle = 0;
    int32_t next_trigger_time = 0;

    // just finished first bit
    if (output_state == 0)
    {
        next_trigger_time = curr_cmd.period / 2;
        set_motor_throttle = curr_cmd.throttle_1;
    }
    // just finished full-half cycle
    else if (output_state == 1 && curr_cmd.period / 2 - curr_cmd.t_offset > min_phase_time)
    {
        next_trigger_time = curr_cmd.period / 2 - curr_cmd.t_offset;
        set_motor_throttle = curr_cmd.throttle_0;
    }
    // just finished the whole cycle
    else
    {
        // in case it's here because output_state = 1 but need to skip
        output_state = 2;

        curr_cmd = get_cmd();

        next_trigger_time = (curr_cmd.zero_time + curr_cmd.t_offset + curr_cmd.period * 20 - TimeSync::Get_synced_time()) % curr_cmd.period;
        set_motor_throttle = curr_cmd.throttle_0;

        // if this happened, then it means our trigger time should be before current time, we jump to phase 1 directly
        if (next_trigger_time > curr_cmd.period / 2 + curr_cmd.t_offset)
        {
            next_trigger_time -= curr_cmd.period / 2;
            set_motor_throttle = curr_cmd.throttle_1;
            output_state = 0;
        }
        else if (next_trigger_time < min_phase_time)
        {
            next_trigger_time += curr_cmd.period / 2;
            set_motor_throttle = curr_cmd.throttle_1;
            output_state = 0;
        }
    }

    // set LED
    digitalWrite(LED_R, (output_state == 0) != (curr_cmd.throttle_1 > curr_cmd.throttle_0));

    // set throttle and reset timer
    Motor::Set_throttle(set_motor_throttle);

    timerAlarmDisable(hw_tm);
    timerRestart(hw_tm);
    timerAlarmWrite(hw_tm, next_trigger_time, false);
    timerAlarmEnable(hw_tm);

    output_state = (output_state + 1) % 3;
}

void Set_Throttle_Task(void *pvParameters)
{
    auto data_queue = TimeSync::Get_data_queue_handle();

    // // launch task to change motor speed
    // const esp_timer_create_args_t oneshot_timer_callback_args_1 = {
    //     .callback = &Change_throttle_callback,
    //     .arg = nullptr,
    //     .dispatch_method = ESP_TIMER_TASK,
    //     .name = "change throttle"};
    // esp_timer_create(&oneshot_timer_callback_args_1, &tm_handle);

    hw_tm = timerBegin(2, 80, true);
    timerStart(hw_tm);
    timerAttachInterrupt(hw_tm, Change_throttle_callback, RISING);

    // xTaskCreate(WakeTask, "WakeTask", 10000, nullptr, 20, &waketaskhandle);

    while (true)
    {
        // receive data
        uint8_t temp_data[TimeSync::max_data_len] = {0};
        xQueueReceive(data_queue, temp_data, portMAX_DELAY);
        uint8_t temp_data_len = temp_data[0];

        if (temp_data_len != sizeof(Command))
        {
            continue;
        }

        // update buffer
        Command cmd_info;
        memcpy(cmd_info.raw, temp_data + 1, temp_data_len);
        set_cmd(cmd_info);

        // only do stuff when synced!
        if (TimeSync::SyncedQ())
        {
            // initialize timer interrupt when the rotation speed is up
            // hysterisis is 5 and 10 rotations / s
            if (cmd_info.period > 0 && (cmd_info.period < (ISR_started ? 200000 : 100000)))
            {
                // check if we need to do so or the ISR already can do that for us
                if (!ISR_started)
                {
                    curr_cmd = cmd_info;

                    int64_t next_zero_time = (cmd_info.zero_time + cmd_info.period * 20 - TimeSync::Get_synced_time()) % cmd_info.period;
                    // if too short in timing, we add a full cycle
                    if (next_zero_time < 2000)
                    {
                        next_zero_time = cmd_info.period;
                    }
                    output_state = 2;

                    // // reset only if not close to new/old signal
                    // esp_timer_stop(tm_handle);
                    // esp_timer_start_once(tm_handle, next_zero_time);

                    timerAlarmDisable(hw_tm);
                    timerRestart(hw_tm);
                    timerAlarmWrite(hw_tm, next_zero_time, false);
                    timerAlarmEnable(hw_tm);

                    Motor::Set_throttle((cmd_info.throttle_0 + cmd_info.throttle_1) / 2);

                    ISR_started = true;
                }
            }
            // set to 0 throttle when spinning too slow
            else
            {
                Motor::Set_throttle(0);
                timerAlarmDisable(hw_tm);
                ISR_started = false;
            }
            // // directly use command (throttle_0+throttle_1)/2 when spinning too slow
            // else
            // {
            //     Motor::Set_throttle((cmd_info.throttle_0 + cmd_info.throttle_1) / 2);
            //     // esp_timer_stop(tm_handle);
            //     timerAlarmDisable(hw_tm);
            //     ISR_started = false;
            // }
        }
    }
}

void Send_FB_Task(void *pvParameters)
{
    auto spd_fb_buf = Motor::Get_motor_speed_buffer();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(3));

        while (!spd_fb_buf.Empty_Q())
        {
            auto temp = spd_fb_buf.pop();
            temp.time -= TimeSync::Get_synced_time_offset();
            ESPNOWCOMM::Send(-1, temp.raw, sizeof(Motor::Motor_FB_t));
        }

        // also responsible for fail safe
        constexpr int64_t shutdown_time = 600000; // if no message in this period of time, we shut down the motor.
        if ((last_update_time != 0 && (esp_timer_get_time() - last_update_time >= shutdown_time)) || !TimeSync::SyncedQ())
        {
            Motor::Set_throttle(0);
            // esp_timer_stop(tm_handle);
            timerAlarmDisable(hw_tm);
            last_update_time = 0;
            ISR_started = false;
        }
    }
}

void PrintTask(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        ESPNOWCOMM::Send(-1, (const uint8_t *)debug_str.c_str(), debug_str.length());
    }
}

// AngleMsg angle_info;
// int output_state;
// // if no triggering within a period of time, we reset the trigger value to 0.
// void IRAM_ATTR Send_pulse_ISR_old()
// {
//     digitalWrite(47, HIGH);
//     digitalWrite(LED_R, output_state);
//     if (angle_info.period > 0 && angle_info.period <= 500000)
//     {
//         output_state = 1 - output_state;
//         timerRestart(temp_timer);
//         timerAlarmWrite(temp_timer, angle_info.period / 2, false);
//         timerAlarmEnable(temp_timer);
//     }
// }

// void Set_Throttle_Task_old(void *pvParameters)
// {
//     auto data_queue = TimeSync::Get_data_queue_handle();
//     auto spd_fb_buf = Motor::Get_motor_speed_buffer();
//
//     temp_timer = timerBegin(0, 80, true);
//     timerAttachInterrupt(temp_timer, Send_pulse_ISR_old, true);
//
//     while (true)
//     {
//         // receive data
//         uint8_t temp_data[TimeSync::max_data_len] = {0};
//         xQueueReceive(data_queue, temp_data, portMAX_DELAY);
//         uint8_t temp_data_len = temp_data[0];
//
//         if (temp_data_len != sizeof(AngleMsg))
//         {
//             continue;
//         }
//
//         memcpy(angle_info.raw, temp_data + 1, temp_data_len);
//
//         if (angle_info.period > 0 && angle_info.period <= 500000)
//         {
//             int64_t next_zero_time = (angle_info.zero_time + 5 * angle_info.period - TimeSync::Get_synced_time() - int32_t(LED_angle_offset / (2.0F * M_PI) * float(angle_info.period))) % angle_info.period;
//             if (next_zero_time >= angle_info.period / 2)
//             {
//                 next_zero_time -= angle_info.period;
//                 output_state = 0;
//             }
//             else
//             {
//                 output_state = 1;
//             }
//
//             // reset only if not close to new/old signal
//             if (next_zero_time >= angle_info.period / 8 && next_zero_time <= angle_info.period * 3 / 8)
//             {
//                 timerRestart(temp_timer);
//                 timerAlarmWrite(temp_timer, next_zero_time, false);
//                 timerAlarmEnable(temp_timer);
//             }
//         }
//
//         set_motor_throttle = angle_info.throttle;
//         Motor::Set_throttle(set_motor_throttle);
//
//         vTaskDelay(pdMS_TO_TICKS(2));
//
//         while (!spd_fb_buf.Empty_Q())
//         {
//             auto temp = spd_fb_buf.pop();
//
//             if (spd_fb_buf.Empty_Q())
//             {
//                 SpeedFeedback spdfb;
//
//                 spdfb.time = temp.time - TimeSync::Get_synced_time_offset();
//                 spdfb.throttle = set_motor_throttle;
//                 spdfb.eRPM = temp.eRPM;
//
//                 ESPNOWCOMM::Send(spdfb.raw, sizeof(SpeedFeedback));
//             }
//         }
//     }
// }