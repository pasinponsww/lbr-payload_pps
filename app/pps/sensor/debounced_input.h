#pragma once
/**
 * @file debounced_input.h
 * @brief Digital input that only changes after reading steady for a while
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "dwell.h"
#include "gpio.h"

namespace LBR
{

class DebouncedInput
{
public:
    /**
     * @brief Wrap a pin
     * @param pin input to read, true means active
     * @param debounce_ms time a new level must hold before it is accepted
     */
    DebouncedInput(Gpio& pin, uint32_t debounce_ms);

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
