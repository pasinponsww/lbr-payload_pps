#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include "console/console.h"
#include "motor_support/dc_motor.h"
#include "plant.h"
#include "pps.h"

namespace
{
constexpr uint32_t kPeriodMs = 10;

constexpr char kUsage[] =
    "pps_sim [--fast]\n"
    "  Runs the PPS firmware logic against a simulated mechanism.\n"
    "  Type console commands (help, launch, deploy, retract, abort, reset,\n"
    "  status, stream <ms>) or simulator commands:\n"
    "    sim boost <ms>         launch acceleration for <ms>\n"
    "    sim jam on|off         block the mechanism\n"
    "    sim drvfault on|off    hold the driver fault line\n"
    "    sim reverse on|off     encoder counts backwards\n"
    "    sim moving on|off      IMU never reads at rest\n"
    "    sim deadswitch on|off  extended limit switch never closes\n"
    "  --fast: no real-time pacing, time only advances with 'wait <ms>'\n";

LBR::Motor& initialised(LBR::Motor& motor)
{
    motor.init();
    return motor;
}

class Simulator
{
public:
    Simulator()
        : driver_(plant_.direction, plant_.pwm, plant_.drv_z, plant_.sleep,
                  plant_.fault),
          motor_(driver_, plant_.encoder),
          pps_(initialised(motor_), plant_.retracted_switch,
               plant_.extended_switch),
          console_(plant_.terminal, pps_)
    {
    }

    void tick()
    {
        now_ms_ += kPeriodMs;
        plant_.step(kPeriodMs / 1000.0f);

        LBR::PpsInputs inputs;
        inputs.now_ms = now_ms_;
        if (pps_.wants_imu())
        {
            plant_.imu(inputs.accel, inputs.gyro);
            inputs.imu_valid = true;
        }
        pps_.step(inputs);
        console_.poll(now_ms_);
    }

    void run(uint32_t duration_ms)
    {
        for (uint32_t t = 0; t < duration_ms; t += kPeriodMs)
        {
            tick();
        }
    }

    void line(const std::string& text)
    {
        if (text.rfind("sim ", 0) != 0)
        {
            for (const char c : text)
            {
                plant_.terminal.rx.push_back(static_cast<uint8_t>(c));
            }
            plant_.terminal.rx.push_back('\n');
            return;
        }

        std::istringstream words(text);
        std::string prefix;
        std::string name;
        std::string value;
        words >> prefix >> name >> value;
        const bool on = value == "on";

        if (name == "boost")
        {
            plant_.boost(static_cast<uint32_t>(
                std::strtoul(value.c_str(), nullptr, 10)));
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
            std::printf("err unknown sim command\n");
            return;
        }
        std::printf("ok sim %s %s\n", name.c_str(), value.c_str());
    }

private:
    LBR::Sim::Plant plant_;
    LBR::Drv8245 driver_;
    LBR::Motor motor_;
    LBR::Pps pps_;
    LBR::Console console_;
    uint32_t now_ms_{0};
};

int run_fast(Simulator& sim)
{
    std::string text;
    while (std::getline(std::cin, text))
    {
        if (text.rfind("wait ", 0) == 0)
        {
            sim.run(static_cast<uint32_t>(
                std::strtoul(text.c_str() + 5, nullptr, 10)));
        }
        else if (!text.empty() && text[0] != '#')
        {
            sim.line(text);
            sim.tick();
        }
    }
    return 0;
}

int run_real_time(Simulator& sim)
{
    std::mutex mutex;
    std::queue<std::string> lines;
    bool closed = false;

    std::thread reader(
        [&]
        {
            std::string text;
            while (std::getline(std::cin, text))
            {
                std::lock_guard<std::mutex> lock(mutex);
                lines.push(text);
            }
            std::lock_guard<std::mutex> lock(mutex);
            closed = true;
        });

    auto next = std::chrono::steady_clock::now();
    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            while (!lines.empty())
            {
                sim.line(lines.front());
                lines.pop();
            }
            if (closed)
            {
                break;
            }
        }
        sim.tick();
        next += std::chrono::milliseconds(kPeriodMs);
        std::this_thread::sleep_until(next);
    }
    reader.join();
    return 0;
}
}  // namespace

int main(int argc, char** argv)
{
    bool fast = false;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--fast") == 0)
        {
            fast = true;
        }
        else
        {
            std::fputs(kUsage, stdout);
            return std::strcmp(argv[i], "--help") == 0 ? 0 : 2;
        }
    }

    Simulator sim;
    return fast ? run_fast(sim) : run_real_time(sim);
}
