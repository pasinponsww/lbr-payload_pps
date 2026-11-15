#pragma once
/**
 * @file sensor_frame.h
 * @brief One control-tick snapshot of every PPS sensor
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "imu_math.h"

namespace LBR
{

/**
 * @struct SensorFrame
 * @brief Sensor readings taken at the same instant
 * @note Produced by SensorManager, consumed by the state machine.
 */
struct SensorFrame
{
    uint32_t now_ms{0};
    bool at_retracted{false};  ///< retracted limit switch closed, debounced
    bool at_extended{false};   ///< extended limit switch closed, debounced
    bool imu_valid{false};     ///< accel and gyro were read this tick
    Vec3 accel{};              ///< m/s^2, gravity included
    Vec3 gyro{};               ///< rad/s
};

}  // namespace LBR
