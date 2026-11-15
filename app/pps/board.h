/**
 * @file board.h
 * @brief Hardware the PPS application needs, provided by the board support
 *        package (see bsp/l476.cc)
 * @author Bex Saw
 * @date 1/13/2026
 */

#pragma once
#include <cstdint>
#include "gpio.h"
#include "motion/motor.h"
#include "sensor/sensors.h"
#include "uart.h"

namespace LBR
{

/**
 * @struct StatusLeds
 * @brief The three debug LEDs on the board
 */
struct StatusLeds
{
    Gpio& red;    ///< on while recovering or safed
    Gpio& green;  ///< on while holding the deployed position
    Gpio& blue;   ///< heartbeat, toggles while the loop is running
};

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
