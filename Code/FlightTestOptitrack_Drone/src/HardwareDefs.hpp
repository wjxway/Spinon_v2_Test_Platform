/**
 * @file HardwareDefs.hpp
 * @brief Hardware pin definitions and resource configurations.
 */

#ifndef HARDWARE_DEFS_HPP
#define HARDWARE_DEFS_HPP

#include "Arduino.h"
#include <array>

// LED pin definitions
#define LED_R 48
#define LED_G 33
#define LED_B 47

// motor input and output
#define M_CTRL_OUT 36
#define M_CTRL_IN 37

// power management
#define PWR_SCL 34
#define PWR_SDA 35

// ESP-NOW settings
std::array<uint8_t, 6> station_MAC = {0xF4, 0x12, 0xFA, 0x88, 0x13, 0xD0};
// array<uint8_t, 6> spinon_MAC = {0x64, 0xE8, 0x33, 0x72, 0x02, 0x94}; // Spinon 21
std::array<uint8_t, 6> spinon_MAC = {0x64, 0xE8, 0x33, 0x72, 0x02, 0xF4}; // Spinon 22
constexpr int WiFi_channel = 14;

// Power monitoring setup
const uint32_t SHUNT_MICRO_OHM = 5000;  // Shunt resistance in Micro-Ohm, 5000 = 5 mOhm
const uint16_t MAXIMUM_AMPS = 10;       // Max expected amps for drone application

#endif

