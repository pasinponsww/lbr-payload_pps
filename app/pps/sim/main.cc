#include <charconv>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <queue>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include "sim.h"

namespace
{
using LBR::Sim::Simulator;

constexpr std::string_view kWaitPrefix = "wait ";
constexpr std::chrono::milliseconds kPeriod{10};

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

// Scripted mode: simulated time only moves on 'wait <ms>' lines
int run_fast(Simulator& sim)
{
    std::string text;
    while (std::getline(std::cin, text))
    {
        const std::string_view line(text);
        if (line.starts_with(kWaitPrefix))
        {
            uint32_t duration_ms = 0;
            std::from_chars(line.data() + kWaitPrefix.size(),
                            line.data() + line.size(), duration_ms);
            sim.run(duration_ms);
        }
        else if (!line.empty() && line.front() != '#')
        {
            sim.line(line);
            sim.tick();
        }
    }
    return 0;
}

// Interactive mode: one control period per 10 ms of wall-clock time
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
        next += kPeriod;
        std::this_thread::sleep_until(next);
    }
    reader.join();
    return 0;
}
}  // namespace

int main(int argc, char** argv)
{
    bool fast = false;
    for (const std::string_view argument :
         std::span<char*>(argv, static_cast<size_t>(argc)).subspan(1))
    {
        if (argument == "--fast")
        {
            fast = true;
        }
        else
        {
            std::fputs(kUsage, stdout);
            return argument == "--help" ? 0 : 2;
        }
    }

    static Simulator sim;
    return fast ? run_fast(sim) : run_real_time(sim);
}
