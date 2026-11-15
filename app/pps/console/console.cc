#include "console.h"
#include <charconv>
#include "line_builder.h"

namespace LBR
{

Console::Console(Uart& uart, Pps& pps)
    : uart_(uart), pps_(pps), last_state_(pps.status().state)
{
}

void Console::poll(uint32_t now_ms)
{
    uint8_t byte = 0;
    while (uart_.read(byte))
    {
        receive(static_cast<char>(byte));
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

void Console::receive(char c)
{
    if (c != '\r' && c != '\n')
    {
        if (length_ < kLineSize)
        {
            line_[length_++] = c;
        }
        else
        {
            overflow_ = true;
        }
        return;
    }

    if (overflow_)
    {
        reply("err", "line too long");
    }
    else if (length_ > 0)
    {
        execute({line_.data(), length_});
    }
    length_ = 0;
    overflow_ = false;
}

bool Console::execute(std::string_view line)
{
    for (const Entry& entry : kCommands)
    {
        if (line == entry.name)
        {
            return send_command(entry);
        }
    }
    if (line == "status")
    {
        return report("status");
    }
    if (line.starts_with(kStreamPrefix))
    {
        return set_stream(line.substr(kStreamPrefix.size()));
    }
    if (line == "help")
    {
        return LineBuilder().text(kHelp).send(uart_);
    }
    return reply("err", "unknown command, try help");
}

bool Console::send_command(const Entry& entry)
{
    if (pps_.command(entry.command))
    {
        return reply("ok", entry.name);
    }
    return LineBuilder()
        .text("err ")
        .text(entry.name)
        .text(" not allowed in ")
        .text(to_string(pps_.status().state))
        .send(uart_);
}

bool Console::set_stream(std::string_view argument)
{
    uint32_t period_ms = 0;
    const auto result = std::from_chars(
        argument.data(), argument.data() + argument.size(), period_ms);
    if (result.ec != std::errc{})
    {
        return reply("err", "stream needs a period in ms");
    }
    stream_ms_ = period_ms;
    return reply("ok", "stream");
}

bool Console::report(std::string_view kind)
{
    const PpsStatus status = pps_.status();
    return LineBuilder()
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
        .send(uart_);
}

bool Console::reply(std::string_view prefix, std::string_view text)
{
    return LineBuilder().text(prefix).text(" ").text(text).send(uart_);
}

}  // namespace LBR
