#pragma once
/**
 * @file pps.h
 * @brief Payload positioning state machine
 * @note Sequence: Standby -> Flight -> Deploying -> Holding, with Retracting
 *       back to stow. This class only sequences: motion lives in Travel,
 *       fault handling in Fdir and Recovery, flight detection in
 *       Flight. Every transition is gated by the sensor frame.
 * @author Bex Saw
 * @date 2026/01/05
 */

#include <cstdint>
#include <string_view>
#include "config.h"
#include "fdir/fdir.h"
#include "fdir/recovery.h"
#include "motion/motor.h"
#include "motion/travel.h"
#include "sensor/flight.h"
#include "sensor/sensors.h"

namespace LBR
{

enum class PpsState : uint8_t
{
    Standby,     ///< waiting for launch
    Flight,      ///< launch seen, waiting for landing
    Deploying,   ///< driving to the extended switch
    Holding,     ///< deployed, braked, watching for drift
    Retracting,  ///< driving to the retracted switch
    Recovery,    ///< handling a fault
    Safed        ///< driver asleep, latched until Reset
};

enum class PpsCommand : uint8_t
{
    None,
    Launch,   ///< Standby -> Flight, as if launch had been detected
    Deploy,   ///< Standby/Flight -> Deploying, skipping flight detection
    Retract,  ///< drive to the retracted switch
    Abort,    ///< any state -> Safed
    Reset     ///< Safed -> Standby, clears the recorded fault
};

/**
 * @struct PpsStatus
 * @brief Read-only view of the state machine for telemetry
 */
struct PpsStatus
{
    PpsState state;
    Fault fault;       ///< most recent fault, None after a reset
    int32_t position;  ///< counts from the retracted switch
    int32_t target;    ///< counts from the retracted switch
    int duty;
    uint8_t retries;
    bool at_retracted;
    bool at_extended;
};

class Pps
{
public:
    /**
     * @brief Build the state machine
     * @param motor axis actuator, already initialised, must outlive this
     * @param config mission configuration
     * @note Starts in Standby with the driver asleep.
     */
    Pps(Motor& motor, const PpsConfig& config);

    /**
     * @brief Queue a command for the next step
     * @param command request to apply
     * @return false if the current state does not allow it
     * @note Only one command is kept; a newer one replaces it.
     */
    bool command(PpsCommand command);

    /**
     * @brief Run one control tick
     * @param frame sensor frame of this tick
     */
    void step(const SensorFrame& frame);

    /**
     * @brief Tell whether the next frame needs IMU data
     * @return true in Standby and Flight, the only states that use it
     */
    bool wants_imu() const;

    /**
     * @brief Snapshot for telemetry
     * @return current state, fault, position and switch readings
     */
    PpsStatus status() const;

private:
    void enter(PpsState next);
    void fail(Fault fault);
    void apply_command();
    bool accepts(PpsCommand command) const;
    void begin_deploy();
    Fault monitor();

    void run_standby();
    void run_flight();
    void run_deploying();
    void run_holding();
    void run_retracting();
    void run_recovery();

    Motor& motor_;
    PpsConfig config_;
    Travel travel_;
    Fdir fdir_;
    Recovery recovery_;
    Flight flight_;

    PpsState state_{PpsState::Standby};
    PpsState resume_{PpsState::Standby};
    PpsCommand pending_{PpsCommand::None};
    SensorFrame frame_{};
    uint32_t last_step_ms_{0};
    float dt_s_{0.0f};
    bool deploy_after_stow_{false};
    bool stow_fallback_used_{false};
};

/**
 * @brief Printable name of a state
 * @param state value to name
 * @return short lower-case name, valid for the life of the program
 */
std::string_view to_string(PpsState state);

}  // namespace LBR
