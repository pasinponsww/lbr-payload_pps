#pragma once
/**
 * @file console.h
 * @brief Line-based command console for the PPS over a UART
 * @note One command per line, one reply per line, plain ASCII so it works
 *       from any serial terminal as well as the host CLI.
 *
 *       Commands: launch, deploy, retract, abort, reset, status,
 *                 stream <ms> (0 stops), help
 *       Replies:  ok <command>
 *                 err <reason>
 *                 status state=.. last_fault=.. pos=.. target=.. duty=..
 *                        retries=.. ret=.. ext=..
 *                 event  (same fields, sent on every state change)
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include "pps.h"
#include "uart.h"

namespace LBR
{

class Console
{
public:
    Console(Uart& uart, Pps& pps);

    /**
     * @brief Handle received bytes and send due telemetry, call every tick
     */
    void poll(uint32_t now_ms);

private:
    static constexpr size_t kLineSize = 32;

    void execute(const char* line);
    bool report(const char* kind);
    bool reply(const char* prefix, const char* text);

    Uart& uart_;
    Pps& pps_;
    std::array<char, kLineSize> line_{};
    size_t length_{0};
    bool overflow_{false};
    PpsState last_state_;
    uint32_t stream_ms_{0};
    uint32_t last_stream_ms_{0};
};

}  // namespace LBR
