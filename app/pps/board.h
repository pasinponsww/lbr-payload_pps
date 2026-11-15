/**
 * @file board.h
 * @brief Hardware the PPS application needs, provided by the board support
 *        package (see pps_bsp/l476_board.cc)
 * @author Bex Saw
 * @date 1/13/2026
 */

#pragma once
#include <cstdint>
#include "motor_support/dc_motor.h"
#include "sensor/sensor_manager.h"
#include "task/indicator_task.h"
#include "uart.h"

namespace LBR
{

/**
 * @struct Board
 * @brief Every piece of hardware the application touches
 */
struct Board
{
    Motor& motor;
    SensorSet sensors;
    Uart& console;
    StatusLeds leds;
};

/**
 * @brief Bring up clocks, pins, peripherals and devices
 * @return true if every device initialised
 */
bool bsp_init();

/**
 * @brief Access the board hardware
 * @return the board, usable after bsp_init()
 */
const Board& get_board();

/**
 * @brief Time since boot
 * @return milliseconds, wraps after about 49 days
 */
uint32_t millis();

}  // namespace LBR
