#pragma once
/**
 * @file pid.h
 * @brief PID loop on position: encoder counts in, signed motor duty out
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>

namespace LBR
{

/**
 * @struct PidGains
 * @brief Tuning for the position loop, error is measured in encoder counts
 */
struct PidGains
{
    float kp{0.6f};               ///< duty per count of error
    float ki{0.4f};               ///< duty per count-second
    float kd{0.03f};              ///< duty per count/second of measured speed
    float integral_limit{30.0f};  ///< clamp on the integral term, in duty
    int max_duty{80};             ///< output saturation
    int min_duty{18};             ///< smallest duty that still moves the motor
    int32_t tolerance{3};         ///< counts of error treated as on target
};

class Pid
{
public:
    /**
     * @brief Build a loop
     * @param gains tuning values
     */
    explicit Pid(const PidGains& gains);

    /**
     * @brief Select a new setpoint and clear the loop history
     * @param target setpoint in counts
     * @param position current position, seeds the speed estimate
     */
    void set_target(int32_t target, int32_t position);

    /**
     * @brief Run one loop iteration
     * @param position measured position in counts
     * @param dt_s time since the previous update in seconds
     * @return signed duty in -max_duty..max_duty, 0 inside the tolerance band
     * @note Outside the band the output is never smaller than min_duty, so
     *       the motor does not sit stalled just short of the target.
     */
    int update(int32_t position, float dt_s);

    /**
     * @brief Compare a position with the setpoint
     * @param position measured position in counts
     * @return true when the position is inside the tolerance band
     */
    bool on_target(int32_t position) const;

    /**
     * @brief Current setpoint
     * @return setpoint in counts
     */
    int32_t target() const;

private:
    PidGains gains_;
    int32_t target_{0};
    int32_t last_position_{0};
    float integral_{0.0f};
};

}  // namespace LBR
