
/**
 * @file st_encoder.h
 * @brief Encoder driver specifics for the STM32L4 series.
 * @author Bex Saw
 * @date 1/27/2026
 */

#pragma once

#include <cstdint>
#include "encoder.h"
#include "stm32l476xx.h"
#include "st_gpio.h"

namespace LBR {
namespace Stml4 {

struct StEncoderParams {
    StGpioParams pin_a_params;
    StGpioParams pin_b_params;
    TIM_TypeDef* timer_base;
};

class HwEncoder {
public:
    explicit HwEncoder(const StEncoderParams& params);
    bool init(void);
    int getTicks() const;
    int getStatus() const;

private:
    HwGpio gpio_a_;
    HwGpio gpio_b_;
    TIM_TypeDef* const timer_base_;
    int _status{0};
};

} // namespace Stml4
} // namespace LBR
