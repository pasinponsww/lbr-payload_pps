#include "motor.h"
#include <algorithm>
#include <cstdlib>

namespace LBR
{

Motor::Motor(Drv8245& drv, Encoder& encoder) : drv_(drv), encoder_(encoder)
{
}

bool Motor::init()
{
    bool ok = drv_.set_speed(0);
    ok = drv_.init() && ok;
    awake_ = true;
    duty_ = 0;
    position_ = 0;
    last_raw_ = static_cast<uint16_t>(encoder_.get_ticks());
    return ok && !encoder_fault();
}

bool Motor::drive(int duty)
{
    duty = std::clamp(duty, -100, 100);
    bool ok = true;
    if (!awake_)
    {
        ok = drv_.set_speed(0) && ok;
        ok = drv_.set_sleep(false) && ok;
        ok = drv_.enable_outputs() && ok;
        awake_ = true;
    }
    if (duty != 0)
    {
        const auto direction = duty > 0 ? Drv8245::Direction::Forward
                                        : Drv8245::Direction::Reverse;
        ok = drv_.set_direction(direction) && ok;
    }
    ok = drv_.set_speed(static_cast<uint16_t>(std::abs(duty))) && ok;
    duty_ = duty;
    return ok;
}

bool Motor::standby()
{
    bool ok = drv_.set_speed(0);
    ok = drv_.enable_coast() && ok;
    ok = drv_.set_sleep(true) && ok;
    awake_ = false;
    duty_ = 0;
    return ok;
}

void Motor::sample()
{
    const uint16_t raw = static_cast<uint16_t>(encoder_.get_ticks());
    position_ += static_cast<int16_t>(raw - last_raw_);
    last_raw_ = raw;
}

int32_t Motor::position() const
{
    return position_;
}

int Motor::duty() const
{
    return duty_;
}

bool Motor::driver_fault() const
{
    return awake_ && drv_.check_fault();
}

bool Motor::encoder_fault() const
{
    return encoder_.get_status() != 0;
}

}  // namespace LBR
