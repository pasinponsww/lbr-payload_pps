#pragma once
/**
 * @file dwell.h
 * @brief Timer for conditions that must hold continuously
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>

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

}  // namespace LBR
