#include "debounced_input.h"

namespace LBR
{

DebouncedInput::DebouncedInput(Gpio& pin, uint32_t debounce_ms)
    : pin_(pin), debounce_ms_(debounce_ms)
{
}

bool DebouncedInput::sample(uint32_t now_ms)
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

bool DebouncedInput::active() const
{
    return active_;
}

}  // namespace LBR
