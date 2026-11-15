#include "rtask.h"

namespace LBR
{

Rtask::Rtask(const Board& board)
    : sensors_(board.sensors, kSensorConfig),
      pps_(board.motor, kPpsConfig),
      console_(board.console, pps_),
      control_(sensors_, pps_),
      indicator_(board.leds, pps_)
{
}

bool Rtask::spin(uint32_t now_ms)
{
    if (!started_)
    {
        started_ = true;
        next_ms_ = now_ms;
    }
    if (static_cast<int32_t>(now_ms - next_ms_) < 0)
    {
        return false;
    }
    next_ms_ += kPeriodMs;

    control_.run(now_ms);
    console_.poll(now_ms);
    indicator_.run();
    return true;
}

bool Rtask::abort()
{
    return pps_.command(PpsCommand::Abort);
}

}  // namespace LBR
