/**
 * @file uart.h
 * @brief UART driver interface
 * @author Bex Saw
 * @date 2026/11/15
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include <span>

namespace LBR
{
/**
 * @class Uart
 * @brief Non-blocking byte stream over a UART
 */
class Uart
{
public:
    /**
     * @brief Take one received byte
     * @param byte filled with the oldest waiting byte
     * @return true if a byte was waiting, false otherwise
     */
    virtual bool read(uint8_t& byte) = 0;

    /**
     * @brief Queue bytes for transmission, all or nothing
     * @param data bytes to send
     * @return true if every byte was queued, false if they did not fit
     */
    virtual bool write(std::span<const uint8_t> data) = 0;

    ~Uart() = default;
};
}  // namespace LBR
