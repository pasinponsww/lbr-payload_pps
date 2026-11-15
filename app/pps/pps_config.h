#pragma once
/**
 * @file pps_config.h
 * @brief Mission constants for the payload positioning system
 * @note Everything that depends on the mechanism or needs bench tuning is
 *       reachable from here; the state machine itself carries no numbers.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "control/travel.h"
#include "fdir/fdir.h"
#include "fdir/recovery.h"
#include "sensor/flight_detector.h"

namespace LBR
{

/**
 * @struct PpsConfig
 * @brief Complete configuration of the payload state machine
 */
struct PpsConfig
{
    TravelConfig travel{};              ///< axis geometry and motion
    FlightConfig flight{};              ///< launch and landing detection
    FdirConfig fdir{};                  ///< fault detection and retry policy
    RecoveryConfig recovery{};          ///< recovery sequence
    uint32_t deploy_budget_ms{8000};    ///< time allowed for a deploy
    uint32_t retract_budget_ms{10000};  ///< time allowed for a retract
};

}  // namespace LBR
