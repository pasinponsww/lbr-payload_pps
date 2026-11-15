#include "st_usart.h"

namespace LBR
{
namespace Stml4
{

HwUsart::HwUsart(const StUsartParams& params)
    : base_addr_(params.base_addr), pclk_hz_(params.pclk_hz), baud_(params.baud)
{
}

bool HwUsart::init()
{
    const int8_t slot = index();
    if (slot < 0 || baud_ == 0)
    {
        return false;
    }

    base_addr_->CR1 = 0;
    base_addr_->BRR = (pclk_hz_ + baud_ / 2u) / baud_;
    base_addr_->CR1 =
        USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;

    instances_[slot] = this;
    NVIC_EnableIRQ(kIrq[slot]);
    return true;
}

int8_t HwUsart::index() const
{
    if (base_addr_ == USART1)
    {
        return 0;
    }
    if (base_addr_ == USART2)
    {
        return 1;
    }
    return base_addr_ == USART3 ? 2 : -1;
}

bool HwUsart::read(uint8_t& byte)
{
    if (rx_tail_ == rx_head_)
    {
        return false;
    }
    byte = rx_[rx_tail_];
    rx_tail_ = (rx_tail_ + 1u) % kRxSize;
    return true;
}

bool HwUsart::write(std::span<const uint8_t> data)
{
    const uint16_t used = (tx_head_ + kTxSize - tx_tail_) % kTxSize;
    if (data.size() > static_cast<size_t>(kTxSize - 1u - used))
    {
        return false;
    }
    for (const uint8_t byte : data)
    {
        tx_[tx_head_] = byte;
        tx_head_ = (tx_head_ + 1u) % kTxSize;
    }
    if (!data.empty())
    {
        base_addr_->CR1 |= USART_CR1_TXEIE;
    }
    return true;
}

void HwUsart::irq(uint8_t index)
{
    if (index < kInstances && instances_[index] != nullptr)
    {
        instances_[index]->on_interrupt();
    }
}

void HwUsart::on_interrupt()
{
    const uint32_t isr = base_addr_->ISR;

    if (isr & USART_ISR_RXNE)
    {
        const uint8_t byte = static_cast<uint8_t>(base_addr_->RDR);
        const uint16_t next = (rx_head_ + 1u) % kRxSize;
        if (next != rx_tail_)
        {
            rx_[rx_head_] = byte;
            rx_head_ = next;
        }
    }
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
    {
        base_addr_->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    }
    if ((isr & USART_ISR_TXE) && (base_addr_->CR1 & USART_CR1_TXEIE))
    {
        if (tx_tail_ != tx_head_)
        {
            base_addr_->TDR = tx_[tx_tail_];
            tx_tail_ = (tx_tail_ + 1u) % kTxSize;
        }
        else
        {
            base_addr_->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

}  // namespace Stml4
}  // namespace LBR

extern "C"
{
    void USART1_IRQHandler()
    {
        LBR::Stml4::HwUsart::irq(0);
    }
    void USART2_IRQHandler()
    {
        LBR::Stml4::HwUsart::irq(1);
    }
    void USART3_IRQHandler()
    {
        LBR::Stml4::HwUsart::irq(2);
    }
}
