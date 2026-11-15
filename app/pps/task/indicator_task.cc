#include "indicator_task.h"

namespace LBR
{

IndicatorTask::IndicatorTask(const StatusLeds& leds, const Pps& pps)
    : leds_(leds), pps_(pps)
{
}

bool IndicatorTask::run()
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
