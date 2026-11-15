#include "st_encoder.h"

namespace LBR
{
namespace Stml4
{

/**
 * Routing of GPIO pins to their EXTI lines
 */
class ExtiLine
{
public:
    static void route(const GPIO_TypeDef& port, uint8_t line)
    {
        const uint32_t shift = (line % 4u) * 4u;
        volatile uint32_t& exticr = SYSCFG->EXTICR[line / 4u];
        exticr = (exticr & ~(0xFu << shift)) | (port_index(port) << shift);

        EXTI->RTSR1 |= 1u << line;
        EXTI->FTSR1 |= 1u << line;
        EXTI->PR1 = 1u << line;
        EXTI->IMR1 |= 1u << line;
        NVIC_EnableIRQ(irq(line));
    }

private:
    static constexpr uint32_t kPortSpacing = 0x400u;

    static uint32_t port_index(const GPIO_TypeDef& port)
    {
        return (reinterpret_cast<uint32_t>(&port) - GPIOA_BASE) / kPortSpacing;
    }

    static IRQn_Type irq(uint8_t line)
    {
        if (line <= 4)
        {
            return static_cast<IRQn_Type>(EXTI0_IRQn + line);
        }
        return line <= 9 ? EXTI9_5_IRQn : EXTI15_10_IRQn;
    }
};

HwEncoder::HwEncoder(const StEncoderParams& params)
    : gpio_a_(params.pin_a_params),
      gpio_b_(params.pin_b_params),
      port_a_(params.pin_a_params.base_addr),
      port_b_(params.pin_b_params.base_addr),
      pin_a_(params.pin_a_params.pin_num),
      pin_b_(params.pin_b_params.pin_num),
      direction_(params.invert ? -1 : 1)
{
}

bool HwEncoder::init()
{
    if (!gpio_a_.init() || !gpio_b_.init())
    {
        return false;
    }

    last_state_ = read_state();
    count_ = 0;
    skips_ = 0;
    instance_ = this;

    ExtiLine::route(*port_a_, pin_a_);
    ExtiLine::route(*port_b_, pin_b_);
    initialized_ = true;
    return true;
}

int HwEncoder::get_ticks() const
{
    return static_cast<int>(count_);
}

int HwEncoder::get_status() const
{
    if (!initialized_)
    {
        return 1;
    }
    return skips_ > kMaxSkips ? 2 : 0;
}

void HwEncoder::irq()
{
    if (instance_ != nullptr)
    {
        instance_->on_edge();
    }
}

uint8_t HwEncoder::read_state() const
{
    return static_cast<uint8_t>((((port_a_->IDR >> pin_a_) & 1u) << 1) |
                                ((port_b_->IDR >> pin_b_) & 1u));
}

void HwEncoder::on_edge()
{
    EXTI->PR1 = (1u << pin_a_) | (1u << pin_b_);

    const uint8_t state = read_state();
    const int8_t step = kStep[(last_state_ << 2) | state];
    last_state_ = state;

    if (step == kSkip)
    {
        skips_ = skips_ + 1;
    }
    else
    {
        count_ = count_ + step * direction_;
    }
}

}  // namespace Stml4
}  // namespace LBR

extern "C"
{
    void EXTI0_IRQHandler()
    {
        LBR::Stml4::HwEncoder::irq();
    }
    void EXTI1_IRQHandler()
    {
        LBR::Stml4::HwEncoder::irq();
    }
    void EXTI2_IRQHandler()
    {
        LBR::Stml4::HwEncoder::irq();
    }
    void EXTI3_IRQHandler()
    {
        LBR::Stml4::HwEncoder::irq();
    }
    void EXTI4_IRQHandler()
    {
        LBR::Stml4::HwEncoder::irq();
    }
    void EXTI9_5_IRQHandler()
    {
        LBR::Stml4::HwEncoder::irq();
    }
    void EXTI15_10_IRQHandler()
    {
        LBR::Stml4::HwEncoder::irq();
    }
}
