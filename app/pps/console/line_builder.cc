#include "line_builder.h"
#include <charconv>

namespace LBR
{

LineBuilder& LineBuilder::text(std::string_view text)
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

LineBuilder& LineBuilder::number(int32_t value)
{
    std::array<char, 12> digits{};
    const auto result =
        std::to_chars(digits.data(), digits.data() + digits.size(), value);
    return text(
        {digits.data(), static_cast<size_t>(result.ptr - digits.data())});
}

bool LineBuilder::send(Uart& uart)
{
    for (const char c : kLineEnd)
    {
        data_[size_++] = static_cast<uint8_t>(c);
    }
    const std::span<const uint8_t> line(data_.data(), size_);
    size_ = 0;
    return uart.write(line);
}

}  // namespace LBR
