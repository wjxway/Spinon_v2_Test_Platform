/**
 * @file motor.hpp
 * @brief motor control header
 */
#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

#include "Arduino.h"
#include <cstdint>
#include <vector>

namespace Motor
{
    // when was last result obtained
    extern int64_t timestamp;
    // encoder value of last result, from 0~32767, 15 bits in total.
    extern uint16_t motor_position;
    // motor speed in degree/s
    extern int16_t motor_velocity;

    enum Rotation_direction
    {
        CLOCKWISE = 0x00,
        COUNTERCLOCKWISE = 0x01
    };
    
    /**
     * @brief Init motor
     *
     * @param TX_pin
     * @param RX_pin
     */
    void Init(int TX_pin, int RX_pin);
    
    /**
     * @brief write and read back through serial
     *
     * @param input byte sequence to send
     * @param response_len length of byte sequence, 0 means unknown
     * @return received response byte sequence
     *
     * @note could be platform specific
     */
    std::vector<uint8_t> Serial_transaction(const std::vector<uint8_t> input, const size_t response_len);

    /**
     * @brief return motor position in radians
     *
     * @return float motor position from 0 to 2pi
     */
    float Current_pos();

    /**
     * @brief (2) Set PID parameters (RAM)
     */
    void Set_PID(uint8_t P_pos, uint8_t I_pos, uint8_t P_vel, uint8_t I_vel, uint8_t P_torque, uint8_t I_torque);

    /**
     * @brief (3) Set PID parameters (ROM)
     */
    void Set_PID_ROM(uint8_t P_pos, uint8_t I_pos, uint8_t P_vel, uint8_t I_vel, uint8_t P_torque, uint8_t I_torque);

    /**
     * @brief (15) completely stop the motor and wipe the motor state/memory
     */
    void Stop();

    /**
     * @brief (16) stop the motor but NOT wipe the motor state/memory
     */
    void Pause();

    /**
     * @brief (17) resume from paused state
     */
    void Resume();

    /**
     * @brief (13) read motor state
     */
    void Read_motor_state();

    /**
     * @brief (18) open loop power control
     *
     * @param power input power from -1000 to 1000
     */
    void Set_power(const int16_t power);

    /**
     * @brief (20) closed loop velocity control
     *
     * @param vel input velocity in int32_t, input unit is 0.01dps/LSB
     */
    void Set_velocity(const int32_t vel);

    /**
     * @brief (21) closed loop multi-loop position control 1
     *
     * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
     */
    void Set_multi_loop_position_1(const int64_t pos);

    /**
     * @brief (22) closed loop multi-loop position control 2
     *
     * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
     * @param max_spd maximum speed in 0.01dps/LSB
     */
    void Set_multi_loop_position_2(const int64_t pos, uint32_t max_spd);

    /**
     * @brief (23) closed loop single-loop position control 1
     *
     * @param pos input single-loop position in uint16_t, input unit is 0.01deg/LSB
     * @param dir rotation direction, could be CLOCKWISE or COUNTERCLOCKWISE
     */
    void Set_single_loop_position_1(const uint16_t pos, const Rotation_direction dir);

    /**
     * @brief (24) closed loop multi-loop position control 2
     *
     * @param pos input multi-loop position in int64_t, input unit is 0.01deg/LSB
     * @param dir rotation direction, could be CLOCKWISE or COUNTERCLOCKWISE
     * @param max_spd maximum speed in 0.01dps/LSB
     */
    void Set_single_loop_position_2(const uint16_t pos, const Rotation_direction dir, uint32_t max_spd);

    /**
     * @brief (25) closed loop incremental position control 1
     *
     * @param inc increment angle in int32_t, input unit is 0.01deg/LSB
     */
    void Set_incremental_position_1(const int32_t inc);

    /**
     * @brief (26) closed loop incremental position control 2
     *
     * @param inc increment angle in int32_t, input unit is 0.01deg/LSB
     * @param max_spd maximum speed in 0.01dps/LSB
     */
    void Set_incremental_position_2(const int32_t inc, uint32_t max_spd);
}

#endif
