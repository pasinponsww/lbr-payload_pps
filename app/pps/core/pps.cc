#include "pps.h"

namespace LBR
{

Pps::Pps(Motor& motor, const PpsConfig& config)
    : motor_(motor),
      config_(config),
      travel_(motor, config.travel),
      fdir_(config.fdir),
      recovery_(motor, config.recovery),
      flight_(config.flight)
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

void Pps::step(const SensorFrame& frame)
{
    frame_ = frame;
    dt_s_ = static_cast<float>(frame.now_ms - last_step_ms_) / 1000.0f;
    last_step_ms_ = frame.now_ms;

    motor_.sample();
    apply_command();

    const Fault fault = monitor();
    if (fault != Fault::None)
    {
        fail(fault);
        return;
    }

    switch (state_)
    {
        case PpsState::Standby:
            run_standby();
            break;
        case PpsState::Flight:
            run_flight();
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
            travel_.position(),
            travel_.target(),
            motor_.duty(),
            fdir_.retries(),
            frame_.at_retracted,
            frame_.at_extended};
}

void Pps::enter(PpsState next)
{
    state_ = next;

    switch (next)
    {
        case PpsState::Standby:
        case PpsState::Flight:
            motor_.standby();
            flight_.arm(frame_.now_ms);
            break;
        case PpsState::Safed:
            deploy_after_stow_ = false;
            motor_.standby();
            break;
        case PpsState::Deploying:
            travel_.start_extend();
            fdir_.arm(frame_.now_ms, motor_.position(),
                      config_.deploy_budget_ms);
            break;
        case PpsState::Retracting:
            travel_.start_retract();
            fdir_.arm(frame_.now_ms, motor_.position(),
                      config_.retract_budget_ms);
            break;
        case PpsState::Holding:
            travel_.hold();
            fdir_.clear_retries();
            break;
        case PpsState::Recovery:
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

    const Response response = fdir_.respond(fault);

    if (response == Response::Retry)
    {
        const bool stalled = fault == Fault::Stall && duty != 0;
        const int unjam = !stalled ? 0 : (duty > 0 ? -1 : 1);
        recovery_.begin({fdir_.backoff_ms(), unjam}, frame_.now_ms);
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
        recovery_.begin({fdir_.backoff_ms(), 0}, frame_.now_ms);
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
                   (state_ == PpsState::Standby && !frame_.at_retracted);
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
            deploy_after_stow_ = false;
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

    if (frame_.at_retracted)
    {
        travel_.mark_stowed();
        enter(PpsState::Deploying);
        return;
    }

    // Travel is measured from the retracted switch, so find it first
    deploy_after_stow_ = true;
    enter(PpsState::Retracting);
}

Fault Pps::monitor()
{
    if (state_ != PpsState::Deploying && state_ != PpsState::Retracting)
    {
        return Fault::None;
    }
    return fdir_.monitor({frame_.now_ms, motor_.position(), motor_.duty(),
                          motor_.driver_fault() || travel_.command_failed(),
                          motor_.encoder_fault()});
}

void Pps::run_standby()
{
    if (frame_.at_retracted)
    {
        travel_.mark_stowed();
    }
    if (flight_.launched(frame_))
    {
        enter(PpsState::Flight);
    }
}

void Pps::run_flight()
{
    if (flight_.landed(frame_))
    {
        begin_deploy();
    }
}

void Pps::run_deploying()
{
    switch (travel_.extend(frame_, dt_s_))
    {
        case Travel::Progress::Moving:
            break;
        case Travel::Progress::Arrived:
            enter(PpsState::Holding);
            break;
        case Travel::Progress::Mismatch:
            fail(Fault::LimitMismatch);
            break;
    }
}

void Pps::run_holding()
{
    if (motor_.encoder_fault())
    {
        fail(Fault::EncoderFault);
    }
    else if (motor_.driver_fault() || travel_.command_failed())
    {
        fail(Fault::DriverFault);
    }
    else if (travel_.drifted(frame_))
    {
        enter(PpsState::Deploying);
    }
}

void Pps::run_retracting()
{
    switch (travel_.retract(frame_, dt_s_))
    {
        case Travel::Progress::Moving:
            return;
        case Travel::Progress::Mismatch:
            fail(Fault::LimitMismatch);
            return;
        case Travel::Progress::Arrived:
            break;
    }

    const bool redeploy = deploy_after_stow_;
    deploy_after_stow_ = false;
    fdir_.clear_retries();

    if (stow_fallback_used_)
    {
        enter(PpsState::Safed);
    }
    else
    {
        enter(redeploy ? PpsState::Deploying : PpsState::Standby);
    }
}

void Pps::run_recovery()
{
    switch (recovery_.step(frame_.now_ms))
    {
        case Recovery::Outcome::Busy:
            break;
        case Recovery::Outcome::Cleared:
            enter(resume_);
            break;
        case Recovery::Outcome::Failed:
            fail(recovery_.failure());
            break;
    }
}

std::string_view to_string(PpsState state)
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
