#pragma once
/**
 * @file sim.h
 * @brief The PPS application running on the simulated plant
 * @note Uses the same Rtask as the firmware; only the hardware underneath
 *       the Board is simulated.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include <string_view>
#include "bno055_imu.h"
#include "board.h"
#include "core/rtask.h"
#include "drv8245.h"
#include "motion/motor.h"
#include "plant.h"

namespace LBR::Sim
{

class Simulator
{
public:
    /**
     * @brief Build the plant, the board on top of it and the application
     */
    Simulator();

    /**
     * @brief Advance simulated time by one control period
     */
    void tick();

    /**
     * @brief Advance simulated time
     * @param duration_ms time to run, rounded up to whole control periods
     */
    void run(uint32_t duration_ms);

    /**
     * @brief Handle one line of operator input
     * @param line text without the line ending
     * @return true if the line was accepted
     * @note Lines starting with "sim " change the plant; everything else is
     *       typed into the firmware console.
     */
    bool line(std::string_view line);

private:
    static constexpr uint32_t kPeriodMs = 10;
    static constexpr std::string_view kSimPrefix = "sim ";

    bool configure(std::string_view name, std::string_view value);

    Plant plant_;
    Drv8245 driver_;
    Motor motor_;
    Bno055 imu_;
    Board board_;
    Rtask rtask_;
    uint32_t now_ms_{0};
};

}  // namespace LBR::Sim
