#include "pid.h"
#include <algorithm>
#include <cstdlib>

namespace LBR
{

Pid::Pid(const PidGains& gains) : gains_(gains)
{
}

void Pid::set_target(int32_t target, int32_t position)
{
    target_ = target;
    last_position_ = position;
    integral_ = 0.0f;
}

int Pid::update(int32_t position, float dt_s)
{
    const float error = static_cast<float>(target_ - position);
    const float speed =
        dt_s > 0.0f ? static_cast<float>(position - last_position_) / dt_s
                    : 0.0f;
    last_position_ = position;

    if (on_target(position))
    {
        integral_ = 0.0f;
        return 0;
    }

    integral_ = std::clamp(integral_ + gains_.ki * error * dt_s,
                           -gains_.integral_limit, gains_.integral_limit);

    const float out = gains_.kp * error + integral_ - gains_.kd * speed;
    int duty =
        std::clamp(static_cast<int>(out), -gains_.max_duty, gains_.max_duty);

    // Stiction floor: push at least min_duty toward the target
    if (std::abs(duty) < gains_.min_duty)
    {
        duty = error > 0.0f ? gains_.min_duty : -gains_.min_duty;
    }
    return duty;
}

bool Pid::on_target(int32_t position) const
{
    return std::abs(target_ - position) <= gains_.tolerance;
}

int32_t Pid::target() const
{
    return target_;
}

}  // namespace LBR
