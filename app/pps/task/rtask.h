#pragma once
/**
 * @file rtask.h
 * @brief Periodic runner that owns the whole PPS application
 * @note Builds the sensor manager, state machine, console and tasks on top
 *       of a Board and runs them at a fixed period. Everything is a member,
 *       so one statically allocated Rtask is the entire application and
 *       nothing is ever taken from the heap.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "board.h"
#include "console/console.h"
#include "pps.h"
#include "pps_config.h"
#include "sensor/sensor_manager.h"
#include "task/control_task.h"
#include "task/indicator_task.h"

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
     * @brief Run every task once if a control period has elapsed
     * @param now_ms current time
     * @return true if the tasks ran, false if it was not time yet
     * @note Call as often as possible; the period is kept by this class.
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
    static constexpr PpsConfig kPpsConfig{};
    static constexpr SensorConfig kSensorConfig{};

    SensorManager sensors_;
    Pps pps_;
    Console console_;
    ControlTask control_;
    IndicatorTask indicator_;
    uint32_t next_ms_{0};
    bool started_{false};
};

}  // namespace LBR
