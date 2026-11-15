#include "plant.h"
#include <cmath>
#include <cstdio>

namespace LBR::Sim
{

namespace
{
constexpr float kMaxSpeed = 3000.0f;
constexpr float kDriveTau = 0.05f;
constexpr float kCoastTau = 0.30f;
constexpr float kTravel = 931.0f;
constexpr float kGravity = 9.81f;
constexpr float kBoostAccel = 40.0f;
}  // namespace

bool Pin::toggle()
{
    level = !level;
    return true;
}

bool Pin::set(const bool active)
{
    level = active;
    return true;
}

bool Pin::read()
{
    return level;
}

bool PwmChannel::set_freq(uint32_t)
{
    return true;
}

bool PwmChannel::set_duty_cycle(uint8_t duty_cycle)
{
    duty = duty_cycle;
    return true;
}

int EncoderCount::get_ticks() const
{
    return ticks;
}

int EncoderCount::get_status() const
{
    return status;
}

bool Terminal::read(uint8_t& byte)
{
    if (rx.empty())
    {
        return false;
    }
    byte = rx.front();
    rx.pop_front();
    return true;
}

bool Terminal::write(std::span<const uint8_t> data)
{
    const size_t written = std::fwrite(data.data(), 1, data.size(), stdout);
    std::fflush(stdout);
    return written == data.size();
}

Plant::Plant()
{
    step(0.0f);
}

void Plant::step(float dt_s)
{
    const bool bridge_on = sleep.level && drv_z.level && !driver_fault;
    const float command =
        bridge_on ? (direction.level ? 1.0f : -1.0f) * pwm.duty / 100.0f : 0.0f;
    const float tau = bridge_on ? kDriveTau : kCoastTau;

    velocity_ += (command * kMaxSpeed - velocity_) * dt_s / tau;
    if (jammed)
    {
        velocity_ = 0.0f;
    }
    position_ += velocity_ * dt_s;

    // Hard stops at both ends of travel
    if (position_ < 0.0f || position_ > kTravel)
    {
        position_ = position_ < 0.0f ? 0.0f : kTravel;
        velocity_ = 0.0f;
    }

    const int count = static_cast<int>(std::lround(position_));
    encoder.ticks = encoder_reversed ? -count : count;
    retracted_switch.level = position_ <= 1.0f;
    extended_switch.level = !dead_switch && position_ >= kTravel - 1.0f;
    fault.level = !(driver_fault && sleep.level);

    if (boost_s_ > 0.0f)
    {
        boost_s_ -= dt_s;
    }
}

void Plant::imu(Vec3& accel, Vec3& gyro) const
{
    accel = {0.0f, 0.0f, boost_s_ > 0.0f ? kBoostAccel : kGravity};
    gyro = {moving_base ? 1.0f : 0.0f, 0.0f, 0.0f};
}

void Plant::boost(uint32_t duration_ms)
{
    boost_s_ = static_cast<float>(duration_ms) / 1000.0f;
}

}  // namespace LBR::Sim
