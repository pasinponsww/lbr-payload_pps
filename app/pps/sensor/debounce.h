#pragma once
/**
 * @file debounce.h
 * @brief Hold timer, and a digital input that only changes after reading
 *        steady for a while
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "gpio.h"

namespace LBR
{

class Dwell
{
public:
    /**
     * @brief Feed the current value of the condition
     * @param condition value of the condition this tick
     * @param now_ms current time
     * @param for_ms how long the condition has to stay true
     * @return true once the condition has been true for at least for_ms
     * @note Any false sample restarts the timer.
     */
    bool held(bool condition, uint32_t now_ms, uint32_t for_ms);

    /**
     * @brief Forget the running timer
     */
    void reset();

private:
    bool running_{false};
    uint32_t since_ms_{0};
};

class Debounce
{
public:
    /**
     * @brief Wrap a pin
     * @param pin input to read, true means active
     * @param debounce_ms time a new level must hold before it is accepted
     */
    Debounce(Gpio& pin, uint32_t debounce_ms);

    /**
     * @brief Read the pin and update the debounced state
     * @param now_ms current time
     * @return the debounced state after this sample
     * @note The first sample is taken as the state without waiting.
     */
    bool sample(uint32_t now_ms);

    /**
     * @brief Last debounced state
     * @return true while the input is considered active
     */
    bool active() const;

private:
    Gpio& pin_;
    const uint32_t debounce_ms_;
    Dwell change_{};
    bool active_{false};
    bool primed_{false};
};

}  // namespace LBR
