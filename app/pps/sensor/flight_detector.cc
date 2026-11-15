#include "flight_detector.h"
#include <cmath>

namespace LBR
{

FlightDetector::FlightDetector(const FlightConfig& config) : config_(config)
{
}

void FlightDetector::arm(uint32_t now_ms)
{
    armed_ms_ = now_ms;
    condition_.reset();
}

bool FlightDetector::launched(const SensorFrame& frame)
{
    const bool boosting =
        frame.imu_valid && magnitude(frame.accel) >= config_.launch_accel;
    return condition_.held(boosting, frame.now_ms, config_.launch_ms);
}

bool FlightDetector::landed(const SensorFrame& frame)
{
    const bool at_rest = frame.imu_valid &&
                         std::fabs(magnitude(frame.accel) - kGravity) <=
                             config_.rest_accel_tol &&
                         magnitude(frame.gyro) <= config_.rest_gyro;

    return condition_.held(at_rest, frame.now_ms, config_.landed_ms) ||
           frame.now_ms - armed_ms_ >= config_.max_flight_ms;
}

float FlightDetector::magnitude(const Vec3& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

}  // namespace LBR
