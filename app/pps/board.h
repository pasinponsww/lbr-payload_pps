/**
 * @file board.h
 * @brief Hardware the PPS application needs, provided by the board support
 *        package (see pps_bsp/l476_board.cc)
 * @author Bex Saw
 * @date 1/13/2026
 */

#pragma once
#include <cstdint>
#include "gpio.h"
#include "imu_math.h"
#include "motor_support/dc_motor.h"
#include "uart.h"

namespace LBR
{

struct Board
{
    Motor& motor;
    Gpio& limit_retracted;
    Gpio& limit_extended;
    Uart& console;
    Gpio& led_red;
    Gpio& led_green;
    Gpio& led_blue;
};

/**
 * @brief Bring up clocks, pins, peripherals and devices
 * @return true if every device initialised
 */
bool bsp_init();

/** @brief Board hardware, valid after bsp_init() */
Board& get_board();

/** @brief Milliseconds since boot, wraps after ~49 days */
uint32_t millis();

/**
 * @brief Read acceleration (m/s^2) and angular rate (rad/s) from the IMU
 * @return true if the read succeeded
 */
bool read_imu(Vec3& accel, Vec3& gyro);

}  // namespace LBR
