/**
 * @file encoder.h
 * @brief Encoder driver interface.
 * @author Bex Saw
 * @date 1/27/2026
 *
 */

#pragma once

namespace LBR
{

/**
 * @class Encoder
 * @brief Generic Encoder driver interface (minimal: getTicks, getStatus).
 */
class Encoder
{
public:
    virtual ~Encoder() = default;

    /**
     * @brief Get current encoder ticks (position count).
     * @return Current tick count as int.
     */
    virtual int getTicks() const = 0;

    /**
     * @brief Get encoder driver status.
     * @return 0 for OK, nonzero for error.
     */
    virtual int getStatus() const = 0;
};

}  // namespace LBR
