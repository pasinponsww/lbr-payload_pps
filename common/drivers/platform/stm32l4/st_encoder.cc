

#include "st_encoder.h"

namespace LBR {
namespace Stml4 {

HwEncoder::HwEncoder(const StEncoderParams& params)
    : gpio_a_(params.pin_a_params),
      gpio_b_(params.pin_b_params),
      timer_base_(params.timer_base) {}

bool HwEncoder::init(void) {
    bool ok_a = gpio_a_.init();
    bool ok_b = gpio_b_.init();
    if (!ok_a || !ok_b) {
        _status = 1;
        return false;
    }

    timer_base_->CR1 = 0;
    timer_base_->ARR = 0xFFFF;
    timer_base_->PSC = 0;
    timer_base_->CNT = 0;

    timer_base_->SMCR &= ~TIM_SMCR_SMS;
    timer_base_->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

    timer_base_->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    timer_base_->CCMR1 |= (0x01 << TIM_CCMR1_CC1S_Pos) | (0x01 << TIM_CCMR1_CC2S_Pos);

    timer_base_->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    timer_base_->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    timer_base_->CR1 |= TIM_CR1_CEN;
    _status = 0;
    return true;
}

int HwEncoder::getTicks() const {
    return static_cast<int>(timer_base_->CNT);
}

int HwEncoder::getStatus() const {
    return _status;
}

} // namespace Stml4
} // namespace LBR
