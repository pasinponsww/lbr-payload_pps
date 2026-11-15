#include "pps.h"
#include <cmath>
#include <cstdlib>

namespace LBR
{

namespace
{
constexpr float kGravity = 9.81f;

float magnitude(const Vec3& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
}  // namespace

bool Pps::Dwell::held(bool condition, uint32_t now_ms, uint32_t for_ms)
{
    if (!condition)
    {
        running = false;
        return false;
    }
    if (!running)
    {
        running = true;
        since_ms = now_ms;
    }
    return now_ms - since_ms >= for_ms;
}

void Pps::LimitSwitch::sample(uint32_t now_ms, uint32_t debounce_ms)
{
    const bool raw = pin.read();
    if (change.held(raw != closed, now_ms, debounce_ms))
    {
        closed = raw;
        change = {};
    }
}

Pps::Pps(Motor& motor, Gpio& retracted_switch, Gpio& extended_switch,
         const PpsConfig& config)
    : motor_(motor),
      retracted_{retracted_switch, retracted_switch.read()},
      extended_{extended_switch, extended_switch.read()},
      config_(config),
      controller_(config.gains),
      fdir_(config.fdir),
      travel_counts_(static_cast<int32_t>(
          std::lround(config.travel_deg / 360.0f * config.counts_per_rev)))
{
    motor_.standby();
}

bool Pps::command(PpsCommand command)
{
    if (!accepts(command))
    {
        return false;
    }
    pending_ = command;
    return true;
}

void Pps::step(const PpsInputs& inputs)
{
    now_ms_ = inputs.now_ms;
    dt_s_ = static_cast<float>(now_ms_ - last_step_ms_) / 1000.0f;
    last_step_ms_ = now_ms_;

    motor_.sample();
    retracted_.sample(now_ms_, config_.switch_debounce_ms);
    extended_.sample(now_ms_, config_.switch_debounce_ms);
    apply_command();

    if (is_moving())
    {
        const Fault fault =
            fdir_.monitor({now_ms_, motor_.position(), motor_.duty(),
                           motor_.driver_fault() || actuator_failed_,
                           motor_.encoder_fault()});
        if (fault != Fault::None)
        {
            fail(fault);
            return;
        }
    }

    switch (state_)
    {
        case PpsState::Standby:
            run_standby(inputs);
            break;
        case PpsState::Flight:
            run_flight(inputs);
            break;
        case PpsState::Deploying:
            run_deploying();
            break;
        case PpsState::Holding:
            run_holding();
            break;
        case PpsState::Retracting:
            run_retracting();
            break;
        case PpsState::Recovery:
            run_recovery();
            break;
        case PpsState::Safed:
            break;
    }
}

bool Pps::wants_imu() const
{
    return state_ == PpsState::Standby || state_ == PpsState::Flight;
}

PpsStatus Pps::status() const
{
    return {state_,
            fdir_.last_fault(),
            motor_.position() - stow_,
            controller_.target() - stow_,
            motor_.duty(),
            fdir_.retries(),
            retracted_.closed,
            extended_.closed};
}

void Pps::enter(PpsState next)
{
    state_ = next;
    state_ms_ = now_ms_;
    condition_ = {};
    seeking_ = false;
    actuator_failed_ = false;

    switch (next)
    {
        case PpsState::Standby:
        case PpsState::Safed:
            deploy_after_stow_ = false;
            motor_.standby();
            break;
        case PpsState::Flight:
            motor_.standby();
            break;
        case PpsState::Deploying:
            controller_.set_target(stow_ + travel_counts_, motor_.position());
            fdir_.arm(now_ms_, motor_.position(), config_.deploy_budget_ms);
            break;
        case PpsState::Retracting:
            controller_.set_target(stow_, motor_.position());
            fdir_.arm(now_ms_, motor_.position(), config_.retract_budget_ms);
            break;
        case PpsState::Holding:
            hold_position_ = motor_.position();
            actuate(0);
            fdir_.clear_retries();
            break;
        case PpsState::Recovery:
            phase_ = RecoveryPhase::Quiesce;
            phase_ms_ = now_ms_;
            break;
    }
}

void Pps::fail(Fault fault)
{
    const int duty = motor_.duty();
    if (state_ != PpsState::Recovery)
    {
        resume_ = state_;
    }
    motor_.standby();

    const Response response = fdir_.respond(fault);
    unjam_direction_ = 0;

    if (response == Response::Retry)
    {
        if (fault == Fault::Stall && duty != 0)
        {
            unjam_direction_ = duty > 0 ? -1 : 1;
        }
        enter(PpsState::Recovery);
        return;
    }

    // Fallback: a deploy that cannot finish is retracted once, anything else safes
    if (response == Response::Fallback && resume_ == PpsState::Deploying &&
        !stow_fallback_used_)
    {
        stow_fallback_used_ = true;
        fdir_.clear_retries();
        resume_ = PpsState::Retracting;
        enter(PpsState::Recovery);
        return;
    }
    enter(PpsState::Safed);
}

bool Pps::accepts(PpsCommand command) const
{
    switch (command)
    {
        case PpsCommand::None:
            return true;
        case PpsCommand::Launch:
            return state_ == PpsState::Standby;
        case PpsCommand::Deploy:
            return state_ == PpsState::Standby || state_ == PpsState::Flight;
        case PpsCommand::Retract:
            return state_ == PpsState::Deploying ||
                   state_ == PpsState::Holding ||
                   (state_ == PpsState::Standby && !retracted_.closed);
        case PpsCommand::Abort:
            return state_ != PpsState::Safed;
        case PpsCommand::Reset:
            return state_ == PpsState::Safed;
    }
    return false;
}

void Pps::apply_command()
{
    const PpsCommand command = pending_;
    pending_ = PpsCommand::None;
    if (!accepts(command))
    {
        return;
    }

    switch (command)
    {
        case PpsCommand::None:
            break;
        case PpsCommand::Launch:
            enter(PpsState::Flight);
            break;
        case PpsCommand::Deploy:
            begin_deploy();
            break;
        case PpsCommand::Retract:
            fdir_.clear_retries();
            enter(PpsState::Retracting);
            break;
        case PpsCommand::Abort:
            enter(PpsState::Safed);
            break;
        case PpsCommand::Reset:
            fdir_.reset();
            stow_fallback_used_ = false;
            enter(PpsState::Standby);
            break;
    }
}

void Pps::begin_deploy()
{
    stow_fallback_used_ = false;
    fdir_.reset();

    if (retracted_.closed)
    {
        stow_ = motor_.position();
        referenced_ = true;
        enter(PpsState::Deploying);
        return;
    }

    // Travel is measured from the retracted switch, so find it first
    enter(PpsState::Retracting);
    deploy_after_stow_ = true;
}

bool Pps::actuate(int duty)
{
    const bool ok = motor_.drive(duty);
    actuator_failed_ = actuator_failed_ || !ok;
    return ok;
}

bool Pps::is_moving() const
{
    return state_ == PpsState::Deploying || state_ == PpsState::Retracting;
}

uint32_t Pps::elapsed(uint32_t since_ms) const
{
    return now_ms_ - since_ms;
}

void Pps::run_standby(const PpsInputs& inputs)
{
    if (retracted_.closed)
    {
        stow_ = motor_.position();
        referenced_ = true;
    }

    const bool boosting =
        inputs.imu_valid && magnitude(inputs.accel) >= config_.launch_accel;
    if (condition_.held(boosting, now_ms_, config_.launch_ms))
    {
        enter(PpsState::Flight);
    }
}

void Pps::run_flight(const PpsInputs& inputs)
{
    const bool at_rest = inputs.imu_valid &&
                         std::fabs(magnitude(inputs.accel) - kGravity) <=
                             config_.rest_accel_tol &&
                         magnitude(inputs.gyro) <= config_.rest_gyro;

    // Timed fallback covers a dead IMU so the payload still deploys
    if (condition_.held(at_rest, now_ms_, config_.landed_ms) ||
        elapsed(state_ms_) >= config_.max_flight_ms)
    {
        begin_deploy();
    }
}

void Pps::run_deploying()
{
    const int32_t position = motor_.position();
    const int32_t travelled = position - stow_;

    if (extended_.closed)
    {
        // The switch must close where the encoder expects the end of travel
        if (retracted_.closed ||
            travelled < travel_counts_ - config_.travel_window)
        {
            fail(Fault::LimitMismatch);
            return;
        }
        enter(PpsState::Holding);
        return;
    }
    if (travelled > travel_counts_ + config_.travel_window)
    {
        fail(Fault::LimitMismatch);
        return;
    }

    // Closed loop over the expected travel, then creep onto the switch
    seeking_ = seeking_ || controller_.on_target(position) ||
               position > controller_.target();
    actuate(seeking_ ? config_.seek_duty : controller_.update(position, dt_s_));
}

void Pps::run_holding()
{
    if (motor_.driver_fault() || motor_.encoder_fault())
    {
        fail(motor_.driver_fault() ? Fault::DriverFault : Fault::EncoderFault);
        return;
    }
    if (!extended_.closed ||
        std::abs(motor_.position() - hold_position_) > config_.hold_drift)
    {
        enter(PpsState::Deploying);
    }
}

void Pps::run_retracting()
{
    const int32_t position = motor_.position();

    if (retracted_.closed)
    {
        stow_ = position;
        referenced_ = true;
        fdir_.clear_retries();

        const bool redeploy = deploy_after_stow_ && !stow_fallback_used_;
        if (stow_fallback_used_)
        {
            enter(PpsState::Safed);
        }
        else
        {
            enter(redeploy ? PpsState::Deploying : PpsState::Standby);
        }
        return;
    }
    if (referenced_ && position < stow_ - config_.travel_window)
    {
        fail(Fault::LimitMismatch);
        return;
    }

    // Without a reference the whole way home is a slow seek
    seeking_ = seeking_ || !referenced_ || controller_.on_target(position) ||
               position < controller_.target();
    actuate(seeking_ ? -config_.seek_duty
                     : controller_.update(position, dt_s_));
}

void Pps::run_recovery()
{
    switch (phase_)
    {
        case RecoveryPhase::Quiesce:
            if (elapsed(phase_ms_) < fdir_.backoff_ms())
            {
                break;
            }
            actuate(0);
            phase_ = unjam_direction_ != 0 ? RecoveryPhase::Unjam
                                           : RecoveryPhase::Verify;
            phase_ms_ = now_ms_;
            break;

        case RecoveryPhase::Unjam:
            actuate(unjam_direction_ * config_.unjam_duty);
            if (elapsed(phase_ms_) >= config_.unjam_ms)
            {
                actuate(0);
                phase_ = RecoveryPhase::Verify;
                phase_ms_ = now_ms_;
            }
            break;

        case RecoveryPhase::Verify:
            if (elapsed(phase_ms_) < config_.verify_ms)
            {
                break;
            }
            if (motor_.encoder_fault())
            {
                fail(Fault::EncoderFault);
            }
            else if (motor_.driver_fault() || actuator_failed_)
            {
                fail(Fault::DriverFault);
            }
            else
            {
                enter(resume_);
            }
            break;
    }
}

const char* to_string(PpsState state)
{
    switch (state)
    {
        case PpsState::Standby:
            return "standby";
        case PpsState::Flight:
            return "flight";
        case PpsState::Deploying:
            return "deploying";
        case PpsState::Holding:
            return "holding";
        case PpsState::Retracting:
            return "retracting";
        case PpsState::Recovery:
            return "recovery";
        case PpsState::Safed:
            return "safed";
    }
    return "unknown";
}

}  // namespace LBR
