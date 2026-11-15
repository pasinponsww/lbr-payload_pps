#pragma once
/**
 * @file travel.h
 * @brief Motion of the payload axis between its two limit switches
 * @note Each move is a closed PID loop on the encoder over the expected
 *       travel, followed by a slow creep until the limit switch at that end
 *       closes. The switch has to agree with the encoder.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "motion/motor.h"
#include "motion/pid.h"
#include "sensor/sensors.h"

namespace LBR
{

/**
 * @struct TravelConfig
 * @brief Geometry and motion parameters of the axis
 */
struct TravelConfig
{
    float counts_per_rev{48.0f * 9.7f};  ///< 48 CPR encoder, 9.7:1 gearbox
    float travel_deg{720.0f};    ///< retracted switch to extended switch
    int32_t travel_window{140};  ///< counts the measured travel may be off by
    int seek_duty{25};           ///< duty used to creep onto a switch
    int32_t hold_drift{12};      ///< counts of drift that end a hold
    PidGains gains{};

    /**
     * @brief Expected travel in encoder counts
     * @return travel_deg converted to counts, rounded to nearest
     */
    constexpr int32_t travel_counts() const
    {
        return static_cast<int32_t>(travel_deg / 360.0f * counts_per_rev +
                                    0.5f);
    }
};

class Travel
{
public:
    enum class Progress : uint8_t
    {
        Moving,   ///< still on the way
        Arrived,  ///< limit switch closed where the encoder expected it
        Mismatch  ///< limit switch and encoder disagree
    };

    /**
     * @brief Build the axis motion
     * @param motor actuator to drive, must outlive this object
     * @param config geometry and motion parameters
     */
    Travel(Motor& motor, const TravelConfig& config);

    /**
     * @brief Take the current position as the stowed reference
     * @note Call while the retracted switch is closed.
     */
    void mark_stowed();

    /**
     * @brief Tell whether the stowed reference is known
     * @return true once mark_stowed() has run
     */
    bool referenced() const;

    /**
     * @brief Begin a move toward the extended switch
     */
    void start_extend();

    /**
     * @brief Begin a move toward the retracted switch
     */
    void start_retract();

    /**
     * @brief Run one tick of an extend move
     * @param frame sensor frame of this tick
     * @param dt_s time since the previous tick in seconds
     * @return Arrived when the extended switch closes inside the expected
     *         window, Mismatch when it closes early, both switches are
     *         closed, or the window is overrun, otherwise Moving
     */
    Progress extend(const SensorFrame& frame, float dt_s);

    /**
     * @brief Run one tick of a retract move
     * @param frame sensor frame of this tick
     * @param dt_s time since the previous tick in seconds
     * @return Arrived when the retracted switch closes, Mismatch when the
     *         stowed position is overrun without it, otherwise Moving
     * @note Arriving re-zeroes the stowed reference. Without a reference
     *       the whole move is a slow seek.
     */
    Progress retract(const SensorFrame& frame, float dt_s);

    /**
     * @brief Brake and remember the position to hold
     * @return true if the motor accepted the command
     */
    bool hold();

    /**
     * @brief Check a hold
     * @param frame sensor frame of this tick
     * @return true if the extended switch opened or the axis moved more
     *         than hold_drift since hold()
     */
    bool drifted(const SensorFrame& frame) const;

    /**
     * @brief Tell whether the motor rejected a command during this move
     * @return true if any drive since the last start or hold failed
     */
    bool command_failed() const;

    /**
     * @brief Position relative to the stowed reference
     * @return encoder counts from the retracted switch
     */
    int32_t position() const;

    /**
     * @brief Setpoint of the current move relative to the stowed reference
     * @return encoder counts from the retracted switch
     */
    int32_t target() const;

private:
    bool drive(int duty);

    Motor& motor_;
    TravelConfig config_;
    Pid pid_;
    int32_t stow_{0};
    int32_t hold_position_{0};
    bool referenced_{false};
    bool seeking_{false};
    bool command_failed_{false};
};

}  // namespace LBR
