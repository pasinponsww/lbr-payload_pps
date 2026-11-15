#pragma once
/**
 * @file pps.h
 * @brief Payload positioning state machine
 * @note Sequence: Standby -> Flight -> Deploying -> Holding, with Retracting
 *       back to stow. The axis runs between two limit switches: the encoder
 *       closes the loop on the expected travel and the switch at each end
 *       confirms arrival. A fault in any moving state goes through Recovery,
 *       which retries, falls back to retracting, or latches Safed.
 * @author Bex Saw
 * @date 2026/01/05
 */

#include <cstdint>
#include "control/pid.h"
#include "fdir/fdir.h"
#include "gpio.h"
#include "imu_math.h"
#include "motor_support/dc_motor.h"
#include "pps_config.h"

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
    Reset     ///< Safed -> Standby, clears the latched fault
};

/**
 * @struct PpsInputs
 * @brief Sensor snapshot handed to the state machine every control tick
 */
struct PpsInputs
{
    uint32_t now_ms{0};
    bool imu_valid{false};
    Vec3 accel{};  ///< m/s^2, gravity included
    Vec3 gyro{};   ///< rad/s
};

/**
 * @struct PpsStatus
 * @brief Read-only view of the state machine for telemetry
 */
struct PpsStatus
{
    PpsState state;
    Fault fault;
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
     * @param motor axis actuator, already initialised
     * @param retracted_switch reads true while the mechanism is fully stowed
     * @param extended_switch reads true while the mechanism is fully deployed
     */
    Pps(Motor& motor, Gpio& retracted_switch, Gpio& extended_switch,
        const PpsConfig& config = {});

    /**
     * @brief Queue a command for the next step
     * @return false if the current state does not allow it
     */
    bool command(PpsCommand command);

    /** @brief Run one control tick */
    void step(const PpsInputs& inputs);

    /** @brief true while the state machine is using IMU data */
    bool wants_imu() const;

    PpsStatus status() const;

private:
    /**
     * @brief Tracks how long a condition has been continuously true
     */
    struct Dwell
    {
        bool running{false};
        uint32_t since_ms{0};

        bool held(bool condition, uint32_t now_ms, uint32_t for_ms);
    };

    /**
     * @brief Debounced limit switch
     */
    struct LimitSwitch
    {
        Gpio& pin;
        bool closed{false};
        Dwell change{};

        void sample(uint32_t now_ms, uint32_t debounce_ms);
    };

    enum class RecoveryPhase : uint8_t
    {
        Quiesce,
        Unjam,
        Verify
    };

    void enter(PpsState next);
    void fail(Fault fault);
    void apply_command();
    bool accepts(PpsCommand command) const;
    void begin_deploy();
    bool actuate(int duty);
    bool is_moving() const;
    uint32_t elapsed(uint32_t since_ms) const;

    void run_standby(const PpsInputs& inputs);
    void run_flight(const PpsInputs& inputs);
    void run_deploying();
    void run_holding();
    void run_retracting();
    void run_recovery();

    Motor& motor_;
    LimitSwitch retracted_;
    LimitSwitch extended_;
    PpsConfig config_;
    Pid controller_;
    Fdir fdir_;

    PpsState state_{PpsState::Standby};
    PpsState resume_{PpsState::Standby};
    PpsCommand pending_{PpsCommand::None};
    RecoveryPhase phase_{RecoveryPhase::Quiesce};

    uint32_t now_ms_{0};
    uint32_t last_step_ms_{0};
    uint32_t state_ms_{0};
    uint32_t phase_ms_{0};
    float dt_s_{0.0f};

    int32_t travel_counts_{0};
    int32_t stow_{0};
    int32_t hold_position_{0};
    int unjam_direction_{0};
    bool referenced_{false};
    bool seeking_{false};
    bool actuator_failed_{false};
    bool deploy_after_stow_{false};
    bool stow_fallback_used_{false};

    Dwell condition_{};
};

/** @brief Short printable name of a state */
const char* to_string(PpsState state);

}  // namespace LBR
