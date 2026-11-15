#include "simulator.h"
#include <charconv>
#include <cstdio>

namespace LBR::Sim
{

Simulator::Simulator()
    : driver_(plant_.direction, plant_.pwm, plant_.drv_z, plant_.sleep,
              plant_.fault),
      motor_(driver_, plant_.encoder),
      imu_(plant_.imu_bus),
      board_{motor_,
             {plant_.retracted_switch, plant_.extended_switch, imu_},
             plant_.terminal,
             {plant_.led_red, plant_.led_green, plant_.led_blue}},
      rtask_((motor_.init(), board_))
{
}

void Simulator::tick()
{
    now_ms_ += kPeriodMs;
    plant_.step(kPeriodMs / 1000.0f);
    rtask_.spin(now_ms_);
}

void Simulator::run(uint32_t duration_ms)
{
    for (uint32_t t = 0; t < duration_ms; t += kPeriodMs)
    {
        tick();
    }
}

bool Simulator::line(std::string_view line)
{
    if (!line.starts_with(kSimPrefix))
    {
        for (const char c : line)
        {
            plant_.terminal.rx.push_back(static_cast<uint8_t>(c));
        }
        plant_.terminal.rx.push_back('\n');
        return true;
    }

    const std::string_view rest = line.substr(kSimPrefix.size());
    const size_t space = rest.find(' ');
    const std::string_view name = rest.substr(0, space);
    const std::string_view value =
        space == std::string_view::npos ? "" : rest.substr(space + 1);

    const bool ok = configure(name, value);
    std::printf("%s sim %.*s %.*s\n", ok ? "ok" : "err",
                static_cast<int>(name.size()), name.data(),
                static_cast<int>(value.size()), value.data());
    return ok;
}

bool Simulator::configure(std::string_view name, std::string_view value)
{
    const bool on = value == "on";

    if (name == "boost")
    {
        uint32_t duration_ms = 0;
        const auto result = std::from_chars(
            value.data(), value.data() + value.size(), duration_ms);
        if (result.ec != std::errc{})
        {
            return false;
        }
        plant_.boost(duration_ms);
    }
    else if (name == "jam")
    {
        plant_.jammed = on;
    }
    else if (name == "drvfault")
    {
        plant_.driver_fault = on;
    }
    else if (name == "reverse")
    {
        plant_.encoder_reversed = on;
    }
    else if (name == "moving")
    {
        plant_.moving_base = on;
    }
    else if (name == "deadswitch")
    {
        plant_.dead_switch = on;
    }
    else
    {
        return false;
    }
    return true;
}

}  // namespace LBR::Sim
