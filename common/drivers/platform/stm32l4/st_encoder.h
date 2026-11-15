/**
 * @file st_encoder.h
 * @brief Quadrature encoder driver for the STM32L4, decoded from two GPIO interrupts
 * @note For encoders routed to pins without a timer encoder function.
 *       Counts every edge of both channels (4x decoding). This driver owns
 *       all EXTI interrupt handlers and supports one instance.
 * @author Bex Saw
 * @date 2026/11/15
 */

#pragma once

#include <cstdint>
#include "encoder.h"
#include "st_gpio.h"
#include "stm32l476xx.h"

namespace LBR
{
namespace Stml4
{

struct StEncoderParams
{
    StGpioParams pin_a_params;
    StGpioParams pin_b_params;
    bool invert;  ///< flip the counting direction
};

class HwEncoder : public Encoder
{
public:
    explicit HwEncoder(const StEncoderParams& params);

    /**
     * @brief Configure the pins, route them to EXTI and enable the interrupts
     * @note GPIO and SYSCFG clocks must already be enabled.
     * @return true if successful
     */
    bool init();

    /** @brief Signed count of edges since init */
    int get_ticks() const override;

    /**
     * @brief Driver health
     * @return 0 OK, 1 not initialised, 2 too many skipped transitions
     */
    int get_status() const override;

    /** @brief Interrupt entry, called from the EXTI handlers */
    static void irq();

private:
    void on_edge();

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
