#include "sensors.h"

namespace LBR
{

Sensors::Sensors(const SensorSet& sensors, const SensorConfig& config)
    : limit_retracted_(sensors.limit_retracted, config.switch_debounce_ms),
      limit_extended_(sensors.limit_extended, config.switch_debounce_ms),
      imu_(sensors.imu)
{
}

const SensorFrame& Sensors::sample(uint32_t now_ms, bool read_imu)
{
    frame_.now_ms = now_ms;
    frame_.at_retracted = limit_retracted_.sample(now_ms);
    frame_.at_extended = limit_extended_.sample(now_ms);
    frame_.imu_valid = false;

    if (read_imu)
    {
        Bno055Data data{};
        if (imu_.read_all(data))
        {
            frame_.accel = data.accel;
            frame_.gyro = data.gyro;
            frame_.imu_valid = true;
        }
    }
    return frame_;
}

const SensorFrame& Sensors::frame() const
{
    return frame_;
}

}  // namespace LBR
