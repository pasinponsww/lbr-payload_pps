#pragma once
/**
 * @file pps_config.h
 * @brief Mission constants for the payload positioning system
 * @note Everything that depends on the mechanism or needs bench tuning lives
 *       here, the state machine itself carries no numbers.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "control/pid.h"
#include "fdir/fdir.h"

namespace LBR
{

struct PpsConfig
{
    /** Encoder counts per output-shaft turn: 48 CPR encoder, 9.7:1 gearbox */
    float counts_per_rev{48.0f * 9.7f};

    /** Expected travel from the retracted switch to the extended switch */
    float travel_deg{720.0f};

    /** Counts the measured travel may differ from the expected travel */
    int32_t travel_window{140};

    /** Slow duty used to creep onto a limit switch */
    int seek_duty{25};

    /** Time budgets per supervised motion */
    uint32_t deploy_budget_ms{8000};
    uint32_t retract_budget_ms{10000};

    /** Drift, in counts, that restarts the deploy while holding */
    int32_t hold_drift{12};

    /** A limit switch must read the same for this long to be believed */
    uint32_t switch_debounce_ms{30};

    /** Launch: acceleration magnitude (m/s^2) held for launch_ms */
    float launch_accel{25.0f};
    uint32_t launch_ms{200};

    /** Landing: |accel| within rest_accel_tol of 1 g and gyro below
     *  rest_gyro (rad/s), both held for landed_ms */
    float rest_accel_tol{1.5f};
    float rest_gyro{0.15f};
    uint32_t landed_ms{5000};

    /** Deploy anyway this long after launch if landing was never detected */
    uint32_t max_flight_ms{600000};

    /** Recovery: reverse nudge used to free a stalled mechanism */
    int unjam_duty{40};
    uint32_t unjam_ms{150};

    /** Recovery: wait after waking the driver before judging its health */
    uint32_t verify_ms{20};

    PidGains gains{};
    FdirConfig fdir{};
};

}  // namespace LBR
