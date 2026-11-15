#pragma once
/**
 * @file line_builder.h
 * @brief Fixed-size builder for one line of console output
 * @note No heap and no printf. Text that does not fit is dropped.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include "uart.h"

namespace LBR
{

class LineBuilder
{
public:
    /**
     * @brief Append text
     * @param text characters to add
     * @return this builder, for chaining
     */
    LineBuilder& text(std::string_view text);

    /**
     * @brief Append a signed decimal number
     * @param value number to add
     * @return this builder, for chaining
     */
    LineBuilder& number(int32_t value);

    /**
     * @brief Terminate the line and queue it on a UART
     * @param uart destination
     * @return true if the whole line was queued
     * @note Appends CR LF. The builder is empty again afterwards.
     */
    bool send(Uart& uart);

private:
    static constexpr size_t kCapacity = 112;
    static constexpr std::string_view kLineEnd = "\r\n";

    std::array<uint8_t, kCapacity> data_{};
    size_t size_{0};
};

}  // namespace LBR
