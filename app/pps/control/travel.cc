#include "travel.h"
#include <cstdlib>

namespace LBR
{

Travel::Travel(Motor& motor, const TravelConfig& config)
    : motor_(motor), config_(config), pid_(config.gains)
{
}

void Travel::mark_stowed()
{
    stow_ = motor_.position();
    referenced_ = true;
}

bool Travel::referenced() const
{
    return referenced_;
}

void Travel::start_extend()
{
    pid_.set_target(stow_ + config_.travel_counts(), motor_.position());
    seeking_ = false;
    command_failed_ = false;
}

void Travel::start_retract()
{
    pid_.set_target(stow_, motor_.position());
    seeking_ = false;
    command_failed_ = false;
}

Travel::Progress Travel::extend(const SensorFrame& frame, float dt_s)
{
    const int32_t position = motor_.position();
    const int32_t travelled = position - stow_;
    const int32_t expected = config_.travel_counts();

    if (frame.at_extended)
    {
        const bool early = travelled < expected - config_.travel_window;
        return frame.at_retracted || early ? Progress::Mismatch
                                           : Progress::Arrived;
    }
    if (travelled > expected + config_.travel_window)
    {
        return Progress::Mismatch;
    }

    // Closed loop over the expected travel, then creep onto the switch
    seeking_ = seeking_ || pid_.on_target(position) || position > pid_.target();
    drive(seeking_ ? config_.seek_duty : pid_.update(position, dt_s));
    return Progress::Moving;
}

Travel::Progress Travel::retract(const SensorFrame& frame, float dt_s)
{
    const int32_t position = motor_.position();

    if (frame.at_retracted)
    {
        mark_stowed();
        return Progress::Arrived;
    }
    if (referenced_ && position < stow_ - config_.travel_window)
    {
        return Progress::Mismatch;
    }

    seeking_ = seeking_ || !referenced_ || pid_.on_target(position) ||
               position < pid_.target();
    drive(seeking_ ? -config_.seek_duty : pid_.update(position, dt_s));
    return Progress::Moving;
}

bool Travel::hold()
{
    hold_position_ = motor_.position();
    command_failed_ = false;
    return drive(0);
}

bool Travel::drifted(const SensorFrame& frame) const
{
    return !frame.at_extended ||
           std::abs(motor_.position() - hold_position_) > config_.hold_drift;
}

bool Travel::command_failed() const
{
    return command_failed_;
}

int32_t Travel::position() const
{
    return motor_.position() - stow_;
}

int32_t Travel::target() const
{
    return pid_.target() - stow_;
}

bool Travel::drive(int duty)
{
    const bool ok = motor_.drive(duty);
    command_failed_ = command_failed_ || !ok;
    return ok;
}

}  // namespace LBR
