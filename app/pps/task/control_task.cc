#include "control_task.h"

namespace LBR
{

ControlTask::ControlTask(SensorManager& sensors, Pps& pps)
    : sensors_(sensors), pps_(pps)
{
}

void ControlTask::run(uint32_t now_ms)
{
    pps_.step(sensors_.sample(now_ms, pps_.wants_imu()));
}

}  // namespace LBR
