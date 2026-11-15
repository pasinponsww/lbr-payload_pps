#include "drv8245.h"

namespace LBR
{

Drv8245::Drv8245(Gpio& dir, Pwm& pwm, Gpio& drv_z, Gpio& sleep, Gpio& fault)
    : dir_(dir), pwm_(pwm), drv_z_(drv_z), sleep_(sleep), fault_(fault)
{
}

bool Drv8245::init()
{
    return enable_outputs() && set_sleep(false);
}

bool Drv8245::set_speed(uint16_t pwm_value)
{
    if (pwm_value > 100)
    {
        pwm_value = 100;
    }
    return pwm_.set_duty_cycle(static_cast<uint8_t>(pwm_value));
}

bool Drv8245::set_direction(Direction dir)
{
    return dir_.set(dir == Direction::Forward);
}

bool Drv8245::enable_coast()
{
    return drv_z_.set(false);
}

bool Drv8245::enable_outputs()
{
    return drv_z_.set(true);
}

bool Drv8245::set_sleep(bool enable)
{
    return sleep_.set(!enable);
}

bool Drv8245::check_fault() const
{
    // nFAULT is active low
    return !fault_.read();
}

}  // namespace LBR
