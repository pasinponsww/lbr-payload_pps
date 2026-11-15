/**
 * @file st_usart.h
 * @brief Interrupt-driven USART driver for STM32L476xx
 * @note 8N1. Receive and transmit go through ring buffers so neither call
 *       blocks. Supports USART1, USART2 and USART3 and owns their handlers.
 * @author Bex Saw
 * @date 2026/11/15
 */

#pragma once

#include <array>
#include <cstdint>
#include "stm32l476xx.h"
#include "uart.h"

namespace LBR
{
namespace Stml4
{

/**
 * @struct StUsartParams
 * @brief Peripheral, its input clock and the wanted baud rate
 */
struct StUsartParams
{
    USART_TypeDef* base_addr;
    uint32_t pclk_hz;  ///< clock feeding the peripheral
    uint32_t baud;
};

class HwUsart : public Uart
{
public:
    /**
     * @brief Hw Constructor
     * @param params peripheral, clock and baud rate
     */
    explicit HwUsart(const StUsartParams& params);

    /**
     * @brief Configure the baud rate and enable the receiver, transmitter
     *        and interrupt
     * @return true if the peripheral is supported and the baud rate is valid
     * @note The peripheral clock and pins must already be set up.
     */
    bool init();

    bool read(uint8_t& byte) override;
    bool write(std::span<const uint8_t> data) override;

    /**
     * @brief Interrupt entry, called from the USARTx handlers
     * @param index 0 for USART1, 1 for USART2, 2 for USART3
     * @note Does nothing for a peripheral that has not been initialised.
     */
    static void irq(uint8_t index);

private:
    static constexpr uint16_t kRxSize = 128;
    static constexpr uint16_t kTxSize = 512;
    static constexpr uint8_t kInstances = 3;
    static constexpr std::array<IRQn_Type, kInstances> kIrq = {
        USART1_IRQn, USART2_IRQn, USART3_IRQn};

    static inline std::array<HwUsart*, kInstances> instances_{};

    void on_interrupt();
    int8_t index() const;

    USART_TypeDef* const base_addr_;
    const uint32_t pclk_hz_;
    const uint32_t baud_;

    std::array<uint8_t, kRxSize> rx_{};
    std::array<uint8_t, kTxSize> tx_{};
    volatile uint16_t rx_head_{0};
    volatile uint16_t rx_tail_{0};
    volatile uint16_t tx_head_{0};
    volatile uint16_t tx_tail_{0};
};

}  // namespace Stml4
}  // namespace LBR
