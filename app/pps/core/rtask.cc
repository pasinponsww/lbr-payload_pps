#include "rtask.h"

namespace LBR
{

Rtask::Rtask(const Board& board)
    : sensors_(board.sensors, kSensorConfig),
      pps_(board.motor, kPpsConfig),
      console_(board.console, pps_),
      leds_(board.leds)
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

    pps_.step(sensors_.sample(now_ms, pps_.wants_imu()));
    console_.poll(now_ms);
    indicate();
    return true;
}

bool Rtask::abort()
{
    return pps_.command(PpsCommand::Abort);
}

bool Rtask::indicate()
{
    const PpsState state = pps_.status().state;
    const bool faulted =
        state == PpsState::Recovery || state == PpsState::Safed;

    bool ok = leds_.red.set(faulted);
    ok = leds_.green.set(state == PpsState::Holding) && ok;
    if (tick_++ % kHeartbeatTicks == 0)
    {
        ok = leds_.blue.toggle() && ok;
    }
    return ok;
}

}  // namespace LBR
