#pragma once
/**
 * @file sensor_manager.h
 * @brief Single owner of every PPS sensor
 * @note All sensors are injected as one set. Each control tick the manager
 *       reads them and hands the rest of the application one SensorFrame,
 *       so nothing else touches sensor hardware.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "bno055_imu.h"
#include "debounced_input.h"
#include "gpio.h"
#include "sensor_frame.h"

namespace LBR
{

/**
 * @struct SensorSet
 * @brief The sensor hardware the PPS depends on
 */
struct SensorSet
{
    Gpio& limit_retracted;  ///< reads true while fully stowed
    Gpio& limit_extended;   ///< reads true while fully deployed
    Bno055& imu;
};

/**
 * @struct SensorConfig
 * @brief Sensor conditioning parameters
 */
struct SensorConfig
{
    uint32_t switch_debounce_ms{30};  ///< steady time before a switch counts
};

class SensorManager
{
public:
    /**
     * @brief Take ownership of sampling for a set of sensors
     * @param sensors hardware to read, must outlive the manager
     * @param config conditioning parameters
     */
    SensorManager(const SensorSet& sensors, const SensorConfig& config);

    /**
     * @brief Read every sensor once
     * @param now_ms current time, stamped into the frame
     * @param read_imu false skips the IMU bus transaction this tick
     * @return the frame for this tick, valid until the next call
     * @note The IMU is only read on request so that a slow or stuck bus
     *       cannot delay the motor loop while the payload is moving.
     */
    const SensorFrame& sample(uint32_t now_ms, bool read_imu);

    /**
     * @brief Frame produced by the last sample
     * @return the most recent frame
     */
    const SensorFrame& frame() const;

private:
    DebouncedInput limit_retracted_;
    DebouncedInput limit_extended_;
    Bno055& imu_;
    SensorFrame frame_{};
};

}  // namespace LBR
