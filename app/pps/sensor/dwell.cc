#include "dwell.h"

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

}  // namespace LBR
