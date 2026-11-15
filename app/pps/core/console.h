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
#include <string_view>
#include "pps.h"
#include "uart.h"

namespace LBR
{

class Console
{
public:
    /**
     * @brief Build the console
     * @param uart byte stream to the operator, must outlive this object
     * @param pps state machine to command and report on
     */
    Console(Uart& uart, Pps& pps);

    /**
     * @brief Handle received bytes and send due telemetry
     * @param now_ms current time
     * @note Call once per control tick, after the state machine has stepped.
     */
    void poll(uint32_t now_ms);

private:
    /**
     * @struct Entry
     * @brief Console word and the command it sends
     */
    struct Entry
    {
        std::string_view name;
        PpsCommand command;
    };

    static constexpr size_t kLineSize = 32;
    static constexpr std::string_view kStreamPrefix = "stream ";
    static constexpr std::string_view kHelp =
        "help launch deploy retract abort reset status stream <ms>";
    static constexpr std::array<Entry, 5> kCommands = {{
        {"launch", PpsCommand::Launch},
        {"deploy", PpsCommand::Deploy},
        {"retract", PpsCommand::Retract},
        {"abort", PpsCommand::Abort},
        {"reset", PpsCommand::Reset},
    }};

    void receive(char c);
    bool execute(std::string_view line);
    bool send_command(const Entry& entry);
    bool set_stream(std::string_view argument);
    bool report(std::string_view kind);
    bool reply(std::string_view prefix, std::string_view text);

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
