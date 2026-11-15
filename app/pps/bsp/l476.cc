#include <cstdint>
#include "bno055_imu.h"
#include "board.h"
#include "drv8245.h"
#include "st_encoder.h"
#include "st_gpio.h"
#include "st_i2c.h"
#include "st_pwm.h"
#include "st_sys_clock.h"
#include "st_usart.h"

namespace LBR
{

namespace
{
using Stml4::GpioMode;
using Stml4::GpioOspeed;
using Stml4::GpioOtype;
using Stml4::GpioPupd;
using Stml4::StGpioParams;

constexpr uint32_t kPclkHz = 4'000'000;
constexpr uint32_t kConsoleBaud = 115'200;
constexpr uint32_t kMotorPwmHz = 20'000;
constexpr uint32_t kI2cTiming = 0x10909CEC;

constexpr uint8_t kAfTim1 = 1;
constexpr uint8_t kAfI2c1 = 4;
constexpr uint8_t kAfUsart1 = 7;

/**
 * Pin parameter presets used by this board
 */
struct Pins
{
    static StGpioParams input(GPIO_TypeDef* port, uint8_t pin, GpioPupd pull)
    {
        return {
            {GpioMode::INPUT, GpioOtype::PUSH_PULL, GpioOspeed::LOW, pull, 0},
            pin,
            port};
    }

    static StGpioParams active_low_input(GPIO_TypeDef* port, uint8_t pin)
    {
        StGpioParams params = input(port, pin, GpioPupd::PULL_UP);
        params.active_low = true;
        return params;
    }

    static StGpioParams output(GPIO_TypeDef* port, uint8_t pin)
    {
        return {{GpioMode::GPOUT, GpioOtype::PUSH_PULL, GpioOspeed::LOW,
                 GpioPupd::NO_PULL, 0},
                pin,
                port};
    }

    static StGpioParams alternate(GPIO_TypeDef* port, uint8_t pin, uint8_t af,
                                  GpioOtype otype = GpioOtype::PUSH_PULL,
                                  GpioPupd pull = GpioPupd::NO_PULL)
    {
        return {
            {GpioMode::ALT_FUNC, otype, GpioOspeed::HIGH, pull, af}, pin, port};
    }
};

// Limit switches, closed to ground: extended LMT_SWT (PA3), retracted (PA5)
Stml4::HwGpio limit_extended(Pins::active_low_input(GPIOA, 3));
Stml4::HwGpio limit_retracted(Pins::active_low_input(GPIOA, 5));

// Motor driver: MTR_PWM1 (PA8, TIM1_CH1), MTR_DIR2 (PA9), DR_FAULT (PC6), MTR_SLP (PC7), DRV_Z (PC8)
Stml4::HwGpio gpio_mtr_pwm1(Pins::alternate(GPIOA, 8, kAfTim1));
Stml4::HwGpio gpio_mtr_dir2(Pins::output(GPIOA, 9));
Stml4::HwGpio gpio_drv_fault(Pins::input(GPIOC, 6, GpioPupd::PULL_UP));
Stml4::HwGpio gpio_mtr_slp(Pins::output(GPIOC, 7));
Stml4::HwGpio gpio_drv_z(Pins::output(GPIOC, 8));

// Spare output: CS_PADC (PC0)
Stml4::HwGpio gpio_cs_padc(Pins::output(GPIOC, 0));

// Debug LEDs: DBG_RED (PC1), DBG_BLUE (PC2), DBG_GREEN (PC3)
Stml4::HwGpio gpio_dbg_red(Pins::output(GPIOC, 1));
Stml4::HwGpio gpio_dbg_blue(Pins::output(GPIOC, 2));
Stml4::HwGpio gpio_dbg_green(Pins::output(GPIOC, 3));

// Console: DBG_USART_TX (PB6), DBG_USART_RX (PB7) on USART1
Stml4::HwGpio gpio_dbg_tx(Pins::alternate(GPIOB, 6, kAfUsart1));
Stml4::HwGpio gpio_dbg_rx(Pins::alternate(GPIOB, 7, kAfUsart1,
                                          GpioOtype::PUSH_PULL,
                                          GpioPupd::PULL_UP));

// IMU bus: I2C1 SCL (PB8), SDA (PB9)
Stml4::HwGpio gpio_i2c_scl(Pins::alternate(GPIOB, 8, kAfI2c1,
                                           GpioOtype::OPEN_DRAIN,
                                           GpioPupd::PULL_UP));
Stml4::HwGpio gpio_i2c_sda(Pins::alternate(GPIOB, 9, kAfI2c1,
                                           GpioOtype::OPEN_DRAIN,
                                           GpioPupd::PULL_UP));

Stml4::HwClock clock;

Stml4::HwPwm motor_pwm({TIM1,
                        1,
                        {Stml4::PwmMode::EDGE_ALIGNED,
                         Stml4::PwmOutputMode::MODE1,
                         Stml4::PwmDir::UPCOUNTING}});

// Encoder: ENC_A_L (PC4), ENC_B_L (PC5)
Stml4::HwEncoder encoder({Pins::input(GPIOC, 4, GpioPupd::NO_PULL),
                          Pins::input(GPIOC, 5, GpioPupd::NO_PULL), false});

Drv8245 motor_driver(gpio_mtr_dir2, motor_pwm, gpio_drv_z, gpio_mtr_slp,
                     gpio_drv_fault);
Motor motor(motor_driver, encoder);

Stml4::HwUsart console_uart({USART1, kPclkHz, kConsoleBaud});

Stml4::HwI2c i2c({I2C1, kI2cTiming});
Bno055 imu(i2c);

Board board{motor,
            {limit_retracted, limit_extended, imu},
            console_uart,
            {gpio_dbg_red, gpio_dbg_green, gpio_dbg_blue}};
}  // namespace

bool bsp_init()
{
    bool ok = clock.init(Stml4::HwClock::configuration::DEFAULT_4MHZ);

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN |
                    RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN;
    RCC->APB2ENR |=
        RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    ok = gpio_dbg_red.init() && ok;
    ok = gpio_dbg_blue.init() && ok;
    ok = gpio_dbg_green.init() && ok;
    ok = gpio_cs_padc.init() && ok;

    ok = limit_extended.init() && ok;
    ok = limit_retracted.init() && ok;
    ok = gpio_mtr_pwm1.init() && ok;
    ok = gpio_mtr_dir2.init() && ok;
    ok = gpio_drv_fault.init() && ok;
    ok = gpio_mtr_slp.init() && ok;
    ok = gpio_drv_z.init() && ok;

    ok = gpio_dbg_tx.init() && ok;
    ok = gpio_dbg_rx.init() && ok;
    ok = console_uart.init() && ok;

    ok = motor_pwm.init() && ok;
    ok = motor_pwm.set_freq(kMotorPwmHz) && ok;
    ok = encoder.init() && ok;
    ok = motor.init() && ok;

    ok = gpio_i2c_scl.init() && ok;
    ok = gpio_i2c_sda.init() && ok;
    ok = i2c.init() && ok;
    imu.init();

    return ok;
}

const Board& get_board()
{
    return board;
}

uint32_t millis()
{
    return HAL_GetTick();
}

}  // namespace LBR
