#include "fdir.h"
#include <cstdlib>

namespace LBR
{

Fdir::Fdir(const FdirConfig& config) : config_(config)
{
}

void Fdir::arm(uint32_t now_ms, int32_t position, uint32_t budget_ms)
{
    armed_ms_ = now_ms;
    budget_ms_ = budget_ms;
    window_ms_ = now_ms;
    window_position_ = position;
    reverse_travel_ = 0;
}

Fault Fdir::monitor(const MotionSample& sample)
{
    if (sample.driver_fault)
    {
        return Fault::DriverFault;
    }
    if (sample.encoder_fault)
    {
        return Fault::EncoderFault;
    }
    if (sample.now_ms - armed_ms_ > budget_ms_)
    {
        return Fault::Timeout;
    }

    // Motion is only judged while the motor is pushed hard enough to move
    if (std::abs(sample.duty) < config_.stall_min_duty)
    {
        window_ms_ = sample.now_ms;
        window_position_ = sample.position;
        return Fault::None;
    }

    const int32_t moved = sample.position - window_position_;
    if (std::abs(moved) >= config_.stall_min_counts)
    {
        const bool with_command = (moved > 0) == (sample.duty > 0);
        reverse_travel_ = with_command ? 0 : reverse_travel_ + std::abs(moved);
        window_ms_ = sample.now_ms;
        window_position_ = sample.position;
        return reverse_travel_ >= config_.reverse_counts ? Fault::WrongDirection
                                                         : Fault::None;
    }

    return sample.now_ms - window_ms_ >= config_.stall_ms ? Fault::Stall
                                                          : Fault::None;
}

Response Fdir::respond(Fault fault)
{
    last_fault_ = fault;
    switch (fault)
    {
        case Fault::EncoderFault:
        case Fault::WrongDirection:
            return Response::Safe;
        case Fault::LimitMismatch:
            return Response::Fallback;
        default:
            break;
    }
    if (retries_ >= config_.max_retries)
    {
        return Response::Fallback;
    }
    ++retries_;
    return Response::Retry;
}

uint32_t Fdir::backoff_ms() const
{
    const uint8_t shift = retries_ > 0 ? retries_ - 1 : 0;
    return config_.backoff_ms << shift;
}

void Fdir::clear_retries()
{
    retries_ = 0;
}

void Fdir::reset()
{
    retries_ = 0;
    last_fault_ = Fault::None;
}

Fault Fdir::last_fault() const
{
    return last_fault_;
}

uint8_t Fdir::retries() const
{
    return retries_;
}

std::string_view to_string(Fault fault)
{
    switch (fault)
    {
        case Fault::None:
            return "none";
        case Fault::DriverFault:
            return "driver";
        case Fault::EncoderFault:
            return "encoder";
        case Fault::Stall:
            return "stall";
        case Fault::WrongDirection:
            return "wrong_direction";
        case Fault::Timeout:
            return "timeout";
        case Fault::LimitMismatch:
            return "limit_mismatch";
    }
    return "unknown";
}

}  // namespace LBR
