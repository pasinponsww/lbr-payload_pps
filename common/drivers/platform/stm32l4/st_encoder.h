/**
 * @file st_encoder.h
 * @brief Quadrature encoder driver for the STM32L4, decoded from two GPIO
 *        interrupts
 * @note For encoders routed to pins without a timer encoder function.
 *       Counts every edge of both channels (4x decoding). This driver owns
 *       all EXTI interrupt handlers and supports one instance.
 * @author Bex Saw
 * @date 2026/11/15
 */

#pragma once

#include <array>
#include <cstdint>
#include "encoder.h"
#include "st_gpio.h"
#include "stm32l476xx.h"

namespace LBR
{
namespace Stml4
{

/**
 * @struct StEncoderParams
 * @brief Pins and counting direction of one quadrature encoder
 */
struct StEncoderParams
{
    StGpioParams pin_a_params;
    StGpioParams pin_b_params;
    bool invert;  ///< flip the counting direction
};

class HwEncoder : public Encoder
{
public:
    /**
     * @brief Hw Constructor
     * @param params channel pins and counting direction
     */
    explicit HwEncoder(const StEncoderParams& params);

    /**
     * @brief Configure the pins, route them to EXTI and enable the interrupts
     * @return true if both pins were configured
     * @note GPIO and SYSCFG clocks must already be enabled.
     */
    bool init();

    /**
     * @brief Read the signed count of edges seen since init
     * @return current count, positive in the forward direction
     */
    int get_ticks() const override;

    /**
     * @brief Report the driver health
     * @return 0 OK, 1 not initialised, 2 too many skipped transitions
     * @note A skipped transition means both channels changed between two
     *       interrupts, so an edge was missed.
     */
    int get_status() const override;

    /**
     * @brief Interrupt entry, called from the EXTI handlers
     * @note Does nothing until an instance has been initialised.
     */
    static void irq();

private:
    static constexpr int8_t kSkip = 2;
    static constexpr uint32_t kMaxSkips = 8;

    /**
     * Count change per transition, indexed by (previous state << 2) | new
     * state, where state = (A << 1) | B
     */
    static constexpr std::array<int8_t, 16> kStep = {
        0, 1, -1, kSkip, -1, 0, kSkip, 1, 1, kSkip, 0, -1, kSkip, -1, 1, 0};

    static inline HwEncoder* instance_ = nullptr;

    void on_edge();
    uint8_t read_state() const;

    HwGpio gpio_a_;
    HwGpio gpio_b_;
    GPIO_TypeDef* const port_a_;
    GPIO_TypeDef* const port_b_;
    const uint8_t pin_a_;
    const uint8_t pin_b_;
    const int8_t direction_;

    volatile int32_t count_{0};
    volatile uint32_t skips_{0};
    uint8_t last_state_{0};
    bool initialized_{false};
};

}  // namespace Stml4
}  // namespace LBR
