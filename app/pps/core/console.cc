#include "console.h"
#include <array>
#include <charconv>
#include <span>

namespace LBR
{

// Fixed-size builder for one line of output, no heap and no printf
class Line
{
public:
    Line& text(std::string_view text);

    Line& number(int32_t value);

    bool send(Uart& uart);

private:
    static constexpr size_t kCapacity = 112;
    static constexpr std::string_view kLineEnd = "\r\n";

    std::array<uint8_t, kCapacity> data_{};
    size_t size_{0};
};

Line& Line::text(std::string_view text)
{
    // Room for the line ending is always kept
    for (const char c : text)
    {
        if (size_ >= kCapacity - kLineEnd.size())
        {
            break;
        }
        data_[size_++] = static_cast<uint8_t>(c);
    }
    return *this;
}

Line& Line::number(int32_t value)
{
    std::array<char, 12> digits{};
    const auto result =
        std::to_chars(digits.data(), digits.data() + digits.size(), value);
    return text(
        {digits.data(), static_cast<size_t>(result.ptr - digits.data())});
}

bool Line::send(Uart& uart)
{
    for (const char c : kLineEnd)
    {
        data_[size_++] = static_cast<uint8_t>(c);
    }
    const std::span<const uint8_t> line(data_.data(), size_);
    size_ = 0;
    return uart.write(line);
}

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
        return Line().text(kHelp).send(uart_);
    }
    return reply("err", "unknown command, try help");
}

bool Console::send_command(const Entry& entry)
{
    if (pps_.command(entry.command))
    {
        return reply("ok", entry.name);
    }
    return Line()
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
        .send(uart_);
}

bool Console::reply(std::string_view prefix, std::string_view text)
{
    return Line().text(prefix).text(" ").text(text).send(uart_);
}

}  // namespace LBR
