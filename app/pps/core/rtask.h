#pragma once
/**
 * @file rtask.h
 * @brief Periodic runner that owns the whole PPS application
 * @note Builds the sensors, state machine and console on top of a Board and
 *       runs them at a fixed period. Everything is a member, so one
 *       statically allocated Rtask is the entire application and nothing is
 *       ever taken from the heap.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "board.h"
#include "config.h"
#include "console.h"
#include "pps.h"
#include "sensor/sensors.h"

namespace LBR
{

class Rtask
{
public:
    /**
     * @brief Build the application on a board
     * @param board hardware to run on, already initialised, must outlive this
     */
    explicit Rtask(const Board& board);

    /**
     * @brief Run one control tick if a control period has elapsed
     * @param now_ms current time
     * @return true if the tick ran, false if it was not time yet
     * @note Call as often as possible; the period is kept by this class.
     *       A tick samples the sensors, steps the state machine, services
     *       the console and refreshes the status LEDs, in that order.
     */
    bool spin(uint32_t now_ms);

    /**
     * @brief Put the payload in the safe state
     * @return true if the state machine accepted the abort
     * @note Used when board bring-up failed; the console still answers.
     */
    bool abort();

private:
    static constexpr uint32_t kPeriodMs = 10;
    static constexpr uint32_t kHeartbeatTicks = 50;
    static constexpr PpsConfig kPpsConfig{};
    static constexpr SensorConfig kSensorConfig{};

    bool indicate();

    Sensors sensors_;
    Pps pps_;
    Console console_;
    StatusLeds leds_;
    uint32_t next_ms_{0};
    uint32_t tick_{0};
    bool started_{false};
};

}  // namespace LBR
