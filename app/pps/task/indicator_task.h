#pragma once
/**
 * @file indicator_task.h
 * @brief Status LEDs driven from the state machine
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "gpio.h"
#include "pps.h"

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

class IndicatorTask
{
public:
    /**
     * @brief Build the task
     * @param leds outputs to drive, must outlive this object
     * @param pps state machine to display
     */
    IndicatorTask(const StatusLeds& leds, const Pps& pps);

    /**
     * @brief Refresh the LEDs, call once per control tick
     * @return true if every LED write succeeded
     */
    bool run();

private:
    static constexpr uint32_t kHeartbeatTicks = 50;

    StatusLeds leds_;
    const Pps& pps_;
    uint32_t tick_{0};
};

}  // namespace LBR
