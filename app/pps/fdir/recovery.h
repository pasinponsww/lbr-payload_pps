#pragma once
/**
 * @file recovery.h
 * @brief Recovery sequence run after a fault interrupts a motion
 * @note Quiesce (driver asleep for the backoff, which also clears latched
 *       driver faults), optionally Unjam (short nudge away from a stall),
 *       then Verify (driver and encoder must read healthy).
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "fdir/fdir.h"
#include "motion/motor.h"

namespace LBR
{

/**
 * @struct RecoveryConfig
 * @brief Timing and effort of the recovery sequence
 */
struct RecoveryConfig
{
    int unjam_duty{40};      ///< duty of the nudge that frees a stall
    uint32_t unjam_ms{150};  ///< length of the nudge
    uint32_t verify_ms{20};  ///< wait after waking the driver before judging
};

class Recovery
{
public:
    enum class Outcome : uint8_t
    {
        Busy,     ///< sequence still running
        Cleared,  ///< hardware reads healthy, the motion may resume
        Failed    ///< hardware still faulted, see failure()
    };

    /**
     * @struct Plan
     * @brief What one recovery attempt should do
     */
    struct Plan
    {
        uint32_t backoff_ms;  ///< quiet time before touching the motor
        int unjam_direction;  ///< +1 or -1 to nudge that way, 0 for no nudge
    };

    /**
     * @brief Build the sequence
     * @param motor actuator to recover, must outlive this object
     * @param config timing and effort
     */
    Recovery(Motor& motor, const RecoveryConfig& config);

    /**
     * @brief Start a recovery attempt and put the driver to sleep
     * @param plan backoff and optional nudge for this attempt
     * @param now_ms current time
     * @return true if the motor accepted the standby command
     */
    bool begin(const Plan& plan, uint32_t now_ms);

    /**
     * @brief Advance the sequence by one control tick
     * @param now_ms current time
     * @return Busy until the sequence ends, then Cleared or Failed
     */
    Outcome step(uint32_t now_ms);

    /**
     * @brief Fault found by the last Verify
     * @return EncoderFault or DriverFault after a Failed outcome
     */
    Fault failure() const;

private:
    enum class Phase : uint8_t
    {
        Quiesce,
        Unjam,
        Verify
    };

    void advance(Phase next, uint32_t now_ms);
    bool drive(int duty);

    Motor& motor_;
    RecoveryConfig config_;
    Plan plan_{0, 0};
    Phase phase_{Phase::Quiesce};
    uint32_t phase_ms_{0};
    bool command_failed_{false};
    Fault failure_{Fault::None};
};

}  // namespace LBR
