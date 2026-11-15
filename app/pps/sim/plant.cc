#include "plant.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace LBR::Sim
{

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

bool ImuBus::mem_read(std::span<uint8_t> data, const uint8_t reg_addr, uint8_t)
{
    if (reg_addr != kDataRegister || data.size() < kGyroOffset + 6)
    {
        return false;
    }
    std::fill(data.begin(), data.end(), uint8_t{0});
    encode(data.subspan(0, 6), accel, kAccelScale);
    encode(data.subspan(kGyroOffset, 6), gyro, kGyroScale);
    return true;
}

bool ImuBus::mem_read(std::span<uint8_t>, const uint16_t, uint8_t)
{
    return false;
}

bool ImuBus::mem_write(std::span<const uint8_t>, const uint8_t, uint8_t)
{
    return true;
}

bool ImuBus::mem_write(std::span<const uint8_t>, const uint16_t, uint8_t)
{
    return false;
}

bool ImuBus::read(std::span<uint8_t>, uint8_t)
{
    return false;
}

bool ImuBus::write(std::span<const uint8_t>, uint8_t)
{
    return false;
}

void ImuBus::encode(std::span<uint8_t> out, const Vec3& v, float scale)
{
    const float axes[3] = {v.x, v.y, v.z};
    for (size_t i = 0; i < 3; ++i)
    {
        const auto raw = static_cast<int16_t>(std::lround(axes[i] * scale));
        out[2 * i] = static_cast<uint8_t>(raw & 0xFF);
        out[2 * i + 1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    }
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
    imu_bus.accel = {0.0f, 0.0f, boost_s_ > 0.0f ? kBoostAccel : kGravity};
    imu_bus.gyro = {moving_base ? 1.0f : 0.0f, 0.0f, 0.0f};
}

void Plant::boost(uint32_t duration_ms)
{
    boost_s_ = static_cast<float>(duration_ms) / 1000.0f;
}

}  // namespace LBR::Sim
