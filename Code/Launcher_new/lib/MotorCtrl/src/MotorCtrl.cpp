/**
 * @file motor.cpp
 * @brief for controlling the Km-tech motors through serial
 *
 * @warning this is NOT thread-safe!
 *
 * @note includes the following commands only:
 * command 15~17: turn off/stop/run motor
 * command 13&18~26: read/control motor position/velocity (same feedback)
 */
#define _USE_MATH_DEFINES
#include "MotorCtrl.hpp"
#include <cmath>
#include <cstring>

using std::vector;

// We only have one motor, so no need to worry about a lot of things.
namespace Motor
{
    constexpr uint8_t Motor_ID = 0x01;

    // when was last result obtained
    int64_t timestamp = 0;
    // encoder value of last result, from 0~65535, 16 bits in total.
    uint16_t motor_position = 0;
    // motor speed in degree/s
    int16_t motor_velocity = 0;

    /**
     * @brief Init motor
     *
     * @param TX_pin
     * @param RX_pin
     */
    void Init(int TX_pin, int RX_pin)
    {
        Serial1.begin(2000000, SERIAL_8N1, RX_pin, TX_pin, false, 5);
    }

    /**
     * @brief write and read back through serial
     *
     * @param input byte sequence to send
     * @param response_len length of byte sequence, 0 means unknown
     * @return received response byte sequence
     *
     * @note could be platform specific
     */
    vector<uint8_t> Serial_transaction(const vector<uint8_t> input, const size_t response_len)
    {
        // clear input serial buffer
        while (Serial1.available())
        {
            Serial1.read();
        }

        // write to serial
        Serial1.write(input.data(), input.size());

        // wait for feedback to arrive
        vector<uint8_t> output(response_len, 0);

        // read to output
        Serial1.readBytes(output.data(), response_len);

        return output;
    }

    /**
     * @brief return motor position in radians
     *
     * @return float motor position from 0 to 2pi
     */
    float Current_pos()
    {
        float rounds = float(motor_position) / 65536.0F + float(esp_timer_get_time() - timestamp) * 1.0e-6 * float(motor_velocity) / 360.0F;
        return (rounds - std::floor(rounds)) * 2.0F * M_PI;
    }

    namespace
    {
        /**
         * @brief compute checksum of in[start] to in[end]
         *
         * @param in input vector
         * @param start starting index
         * @param end ending index
         * @return uint8_t checksum byte
         */
        uint8_t Checksum(const vector<uint8_t> in, const size_t start, const size_t end)
        {
            uint8_t cs = 0;

            for (int i = start; i <= end; i++)
            {
                cs += in[i];
            }

            return cs;
        }

        /**
         * @brief Parse the regular 13 bytes response from the motor. It will
         * set up time stamp, motor_position and motor_velocity.
         *
         * @param response response from the motor
         */
        void Parse_response(const vector<uint8_t> response)
        {
            timestamp = esp_timer_get_time();
            motor_position = (((uint16_t)response[11]) << 8) + response[10];
            motor_velocity = (((int16_t)response[9]) << 8) + response[8];
        }
    }

    /**
     * @brief (2) Set PID parameters (RAM)
     */
    void Set_PID(uint8_t P_pos, uint8_t I_pos, uint8_t P_vel, uint8_t I_vel, uint8_t P_torque, uint8_t I_torque)
    {
        vector<uint8_t> in = {0x3E, 0x31, Motor_ID, 0x06, 0x00, P_pos, I_pos, P_vel, I_vel, P_torque, I_torque, 0x00};
        in[4] = Checksum(in, 0, 3);
        in[11] = Checksum(in, 5, 10);
        Serial_transaction(in, 12);
    }

    /**
     * @brief (3) Set PID parameters (ROM)
     */
    void Set_PID_ROM(uint8_t P_pos, uint8_t I_pos, uint8_t P_vel, uint8_t I_vel, uint8_t P_torque, uint8_t I_torque)
    {
        vector<uint8_t> in = {0x3E, 0x32, Motor_ID, 0x06, 0x00, P_pos, I_pos, P_vel, I_vel, P_torque, I_torque, 0x00};
        in[4] = Checksum(in, 0, 3);
        in[11] = Checksum(in, 5, 10);
        Serial_transaction(in, 12);
    }

    /**
     * @brief (15) completely stop the motor and wipe the motor state/memory
     */
    void Stop()
    {
        vector<uint8_t> in = {0x3E, 0x80, Motor_ID, 0x00, 0x00};
        in[4] = Checksum(in, 0, 3);
        Serial_transaction(in, 5);
    }

    /**
     * @brief (16) stop the motor but NOT wipe the motor state/memory
     */
    void Pause()
    {
        vector<uint8_t> in = {0x3E, 0x81, Motor_ID, 0x00, 0x00};
        in[4] = Checksum(in, 0, 3);
        Serial_transaction(in, 5);
    }

    /**
     * @brief (17) resume from paused state
     */
    void Resume()
    {
        vector<uint8_t> in = {0x3E, 0x88, Motor_ID, 0x00, 0x00};
        in[4] = Checksum(in, 0, 3);
        Serial_transaction(in, 5);
    }

    /**
     * @brief (13) read motor state
     */
    void Read_motor_state()
    {
        vector<uint8_t> in = {0x3E, 0x9C, Motor_ID, 0x00, 0x00};
        in[4] = Checksum(in, 0, 3);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (18) open loop power control
     *
     * @param power input power from -1000 to 1000
     */
    void Set_power(const int16_t power)
    {
        vector<uint8_t> in = {0x3E, 0xA0, Motor_ID, 0x02, 0x00, (uint8_t)(power & 0xFF), (uint8_t)(power >> 8), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[7] = Checksum(in, 5, 6);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (20) closed loop velocity control
     *
     * @param vel input velocity in int32_t, input unit is 0.01dps/LSB
     */
    void Set_velocity(const int32_t vel)
    {
        vector<uint8_t> in = {0x3E, 0xA2, Motor_ID, 0x04, 0x00, (uint8_t)(vel & 0xFF), (uint8_t)((vel >> 8) & 0xFF), (uint8_t)((vel >> 16) & 0xFF), (uint8_t)((vel >> 24) & 0xFF), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[9] = Checksum(in, 5, 8);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (21) closed loop multi-loop position control 1
     *
     * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
     */
    void Set_multi_loop_position_1(const int64_t pos)
    {
        vector<uint8_t> in = {0x3E, 0xA3, Motor_ID, 0x08, 0x00, (uint8_t)(pos & 0xFF), (uint8_t)((pos >> 8) & 0xFF), (uint8_t)((pos >> 16) & 0xFF), (uint8_t)((pos >> 24) & 0xFF), (uint8_t)((pos >> 32) & 0xFF), (uint8_t)((pos >> 40) & 0xFF), (uint8_t)((pos >> 48) & 0xFF), (uint8_t)((pos >> 56) & 0xFF), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[13] = Checksum(in, 5, 12);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (22) closed loop multi-loop position control 2
     *
     * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
     * @param max_spd maximum speed in 0.01dps/LSB
     */
    void Set_multi_loop_position_2(const int64_t pos, uint32_t max_spd)
    {
        vector<uint8_t> in = {0x3E, 0xA4, Motor_ID, 0x0C, 0x00, (uint8_t)(pos & 0xFF), (uint8_t)((pos >> 8) & 0xFF), (uint8_t)((pos >> 16) & 0xFF), (uint8_t)((pos >> 24) & 0xFF), (uint8_t)((pos >> 32) & 0xFF), (uint8_t)((pos >> 40) & 0xFF), (uint8_t)((pos >> 48) & 0xFF), (uint8_t)((pos >> 56) & 0xFF), (uint8_t)(max_spd & 0xFF), (uint8_t)((max_spd >> 8) & 0xFF), (uint8_t)((max_spd >> 16) & 0xFF), (uint8_t)((max_spd >> 24) & 0xFF), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[17] = Checksum(in, 5, 16);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (23) closed loop single-loop position control 1
     *
     * @param pos input single-loop position in uint16_t, input unit is 0.01deg/LSB
     * @param dir rotation direction, could be CLOCKWISE or COUNTERCLOCKWISE
     */
    void Set_single_loop_position_1(const uint16_t pos, const Rotation_direction dir)
    {
        vector<uint8_t> in = {0x3E, 0xA5, Motor_ID, 0x04, 0x00, (uint8_t)dir, (uint8_t)(pos & 0xFF), (uint8_t)((pos >> 8) & 0xFF), 0x00, 0x00};
        in[4] = Checksum(in, 0, 3);
        in[9] = Checksum(in, 5, 8);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (24) closed loop multi-loop position control 2
     *
     * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
     * @param dir rotation direction, could be CLOCKWISE or COUNTERCLOCKWISE
     * @param max_spd maximum speed in 0.01dps/LSB
     */
    void Set_single_loop_position_2(const uint16_t pos, const Rotation_direction dir, uint32_t max_spd)
    {
        vector<uint8_t> in = {0x3E, 0xA6, Motor_ID, 0x08, 0x00, (uint8_t)dir, (uint8_t)(pos & 0xFF), (uint8_t)((pos >> 8) & 0xFF), 0x00, (uint8_t)(max_spd & 0xFF), (uint8_t)((max_spd >> 8) & 0xFF), (uint8_t)((max_spd >> 16) & 0xFF), (uint8_t)((max_spd >> 24) & 0xFF), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[13] = Checksum(in, 5, 12);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (25) closed loop incremental position control 1
     *
     * @param inc increment angle in int32_t, input unit is 0.01deg/LSB
     */
    void Set_incremental_position_1(const int32_t inc)
    {
        vector<uint8_t> in = {0x3E, 0xA7, Motor_ID, 0x04, 0x00, (uint8_t)(inc & 0xFF), (uint8_t)((inc >> 8) & 0xFF), (uint8_t)((inc >> 16) & 0xFF), (uint8_t)((inc >> 24) & 0xFF), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[9] = Checksum(in, 5, 8);
        Parse_response(Serial_transaction(in, 13));
    }

    /**
     * @brief (26) closed loop incremental position control 2
     *
     * @param inc increment angle in int32_t, input unit is 0.01deg/LSB
     * @param max_spd maximum speed in 0.01dps/LSB
     */
    void Set_incremental_position_2(const int32_t inc, uint32_t max_spd)
    {
        vector<uint8_t> in = {0x3E, 0xA8, Motor_ID, 0x08, 0x00, (uint8_t)(inc & 0xFF), (uint8_t)((inc >> 8) & 0xFF), (uint8_t)((inc >> 16) & 0xFF), (uint8_t)((inc >> 24) & 0xFF), (uint8_t)(max_spd & 0xFF), (uint8_t)((max_spd >> 8) & 0xFF), (uint8_t)((max_spd >> 16) & 0xFF), (uint8_t)((max_spd >> 24) & 0xFF), 0x00};
        in[4] = Checksum(in, 0, 3);
        in[13] = Checksum(in, 5, 12);
        Parse_response(Serial_transaction(in, 13));
    }
};
