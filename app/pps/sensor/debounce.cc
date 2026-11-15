#include "debounce.h"

namespace LBR
{

bool Dwell::held(bool condition, uint32_t now_ms, uint32_t for_ms)
{
    if (!condition)
    {
        running_ = false;
        return false;
    }
    if (!running_)
    {
        running_ = true;
        since_ms_ = now_ms;
    }
    return now_ms - since_ms_ >= for_ms;
}

void Dwell::reset()
{
    running_ = false;
}

Debounce::Debounce(Gpio& pin, uint32_t debounce_ms)
    : pin_(pin), debounce_ms_(debounce_ms)
{
}

bool Debounce::sample(uint32_t now_ms)
{
    const bool raw = pin_.read();
    if (!primed_)
    {
        primed_ = true;
        active_ = raw;
    }
    else if (change_.held(raw != active_, now_ms, debounce_ms_))
    {
        active_ = raw;
        change_.reset();
    }
    return active_;
}

bool Debounce::active() const
{
    return active_;
}

}  // namespace LBR
