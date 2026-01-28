/**
 * @file board.h
 * @brief PPS driver interface
 * @author Bex Saw
 * @date 1/13/2026
 */

#pragma once
#include "i2c.h"
#include "gpio.h"
#include "bno055_imu.h"
#include "drv8245.h"
#include "motor_support/dc_motor.h"

namespace LBR
{


struct Board {
    I2c& i2c;
    Gpio& gpio;
    Bno055Data imu;
    Motor* motor;
};

// Implementations are platform-specific (see l476_board.cc)
bool bsp_init();
Board& get_board();

}  // namespace LBR
