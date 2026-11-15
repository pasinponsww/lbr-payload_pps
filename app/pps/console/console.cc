#include "console.h"
#include <cstdlib>
#include <cstring>

namespace LBR
{

namespace
{
struct Entry
{
    const char* name;
    PpsCommand command;
};

constexpr Entry kCommands[] = {
    {"launch", PpsCommand::Launch},   {"deploy", PpsCommand::Deploy},
    {"retract", PpsCommand::Retract}, {"abort", PpsCommand::Abort},
    {"reset", PpsCommand::Reset},
};

constexpr char kHelp[] =
    "help launch deploy retract abort reset status stream <ms>\r\n";

/**
 * Fixed-size line builder, silently truncates when full
 */
class Line
{
public:
    Line& text(const char* s)
    {
        while (*s != '\0' && size_ < sizeof(data_))
        {
            data_[size_++] = *s++;
        }
        return *this;
    }

    Line& number(int32_t value)
    {
        char digits[12];
        size_t count = 0;
        uint32_t magnitude = value < 0 ? 0u - static_cast<uint32_t>(value)
                                       : static_cast<uint32_t>(value);
        do
        {
            digits[count++] = static_cast<char>('0' + magnitude % 10u);
            magnitude /= 10u;
        } while (magnitude != 0u);
        if (value < 0)
        {
            digits[count++] = '-';
        }
        while (count > 0 && size_ < sizeof(data_))
        {
            data_[size_++] = digits[--count];
        }
        return *this;
    }

    bool send(Uart& uart)
    {
        return uart.write({reinterpret_cast<const uint8_t*>(data_), size_});
    }

private:
    char data_[112];
    size_t size_{0};
};
}  // namespace

Console::Console(Uart& uart, Pps& pps)
    : uart_(uart), pps_(pps), last_state_(pps.status().state)
{
}

void Console::poll(uint32_t now_ms)
{
    uint8_t byte = 0;
    while (uart_.read(byte))
    {
        const char c = static_cast<char>(byte);
        if (c != '\r' && c != '\n')
        {
            if (length_ < kLineSize - 1)
            {
                line_[length_++] = c;
            }
            else
            {
                overflow_ = true;
            }
            continue;
        }
        line_[length_] = '\0';
        if (overflow_)
        {
            reply("err", "line too long");
        }
        else if (length_ > 0)
        {
            execute(line_.data());
        }
        length_ = 0;
        overflow_ = false;
    }

    const PpsState state = pps_.status().state;
    if (state != last_state_)
    {
        last_state_ = state;
        report("event");
    }
    if (stream_ms_ != 0 && now_ms - last_stream_ms_ >= stream_ms_)
    {
        last_stream_ms_ = now_ms;
        report("status");
    }
}

void Console::execute(const char* line)
{
    for (const Entry& entry : kCommands)
    {
        if (std::strcmp(line, entry.name) == 0)
        {
            if (pps_.command(entry.command))
            {
                reply("ok", entry.name);
            }
            else
            {
                Line()
                    .text("err ")
                    .text(entry.name)
                    .text(" not allowed in ")
                    .text(to_string(pps_.status().state))
                    .text("\r\n")
                    .send(uart_);
            }
            return;
        }
    }

    if (std::strcmp(line, "status") == 0)
    {
        report("status");
    }
    else if (std::strncmp(line, "stream ", 7) == 0)
    {
        stream_ms_ = static_cast<uint32_t>(std::strtoul(line + 7, nullptr, 10));
        reply("ok", "stream");
    }
    else if (std::strcmp(line, "help") == 0)
    {
        uart_.write(
            {reinterpret_cast<const uint8_t*>(kHelp), sizeof(kHelp) - 1});
    }
    else
    {
        reply("err", "unknown command, try help");
    }
}

bool Console::report(const char* kind)
{
    const PpsStatus status = pps_.status();
    return Line()
        .text(kind)
        .text(" state=")
        .text(to_string(status.state))
        .text(" last_fault=")
        .text(to_string(status.fault))
        .text(" pos=")
        .number(status.position)
        .text(" target=")
        .number(status.target)
        .text(" duty=")
        .number(status.duty)
        .text(" retries=")
        .number(status.retries)
        .text(" ret=")
        .number(status.at_retracted ? 1 : 0)
        .text(" ext=")
        .number(status.at_extended ? 1 : 0)
        .text("\r\n")
        .send(uart_);
}

bool Console::reply(const char* prefix, const char* text)
{
    return Line().text(prefix).text(" ").text(text).text("\r\n").send(uart_);
}

}  // namespace LBR
