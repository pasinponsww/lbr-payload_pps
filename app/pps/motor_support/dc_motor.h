#pragma once
/**
 * @file dc_motor.h
 * @brief Brushed DC motor actuator with encoder position feedback
 * @note Non-blocking: every call returns immediately, the caller owns timing.
 * @author Bex Saw
 * @date 2025/12/31
 */

#include <cstdint>
#include "drv8245.h"
#include "encoder.h"

namespace LBR
{

class Motor
{
public:
    /**
     * @brief Build the actuator
     * @param drv motor driver, must outlive this object
     * @param encoder position feedback, must outlive this object
     */
    Motor(Drv8245& drv, Encoder& encoder);

    /**
     * @brief Wake the driver with zero duty and latch the encoder reference
     * @return true if the driver accepted the setup and the encoder is healthy
     */
    bool init();

    /**
     * @brief Apply a signed duty cycle, waking the driver if needed
     * @param duty -100..100, sign selects direction, 0 brakes
     * @return true if the driver accepted every step of the command
     */
    bool drive(int duty);

    /**
     * @brief Release the bridge (Hi-Z) and put the driver to sleep
     * @return true if the driver accepted every step of the command
     * @note A sleep/wake cycle also clears latched driver faults.
     */
    bool standby();

    /**
     * @brief Accumulate encoder movement, call once per control tick
     * @note Wrap-safe for 16-bit counters as long as a tick moves fewer
     *       than 32768 counts.
     */
    void sample();

    /**
     * @brief Accumulated position
     * @return encoder counts since init(), as of the last sample()
     */
    int32_t position() const;

    /**
     * @brief Last commanded duty
     * @return signed duty, 0 while braked or in standby
     */
    int duty() const;

    /**
     * @brief Driver fault line
     * @return true while the driver is awake and asserts nFAULT
     */
    bool driver_fault() const;

    /**
     * @brief Encoder health
     * @return true if the encoder reports an error
     */
    bool encoder_fault() const;

private:
    Drv8245& drv_;
    Encoder& encoder_;
    int32_t position_{0};
    uint16_t last_raw_{0};
    int duty_{0};
    bool awake_{false};
};

}  // namespace LBR
