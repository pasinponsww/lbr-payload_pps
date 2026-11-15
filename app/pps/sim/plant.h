#pragma once
/**
 * @file plant.h
 * @brief Simulated PPS hardware for running the firmware logic on a PC
 * @note Models the DRV8245 pins, a first-order DC motor, the quadrature
 *       encoder, both limit switches and the IMU, with flags to inject faults.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include <deque>
#include "encoder.h"
#include "gpio.h"
#include "imu_math.h"
#include "pwm.h"
#include "uart.h"

namespace LBR::Sim
{

struct Pin : Gpio
{
    bool level{false};

    bool toggle() override;
    bool set(const bool active) override;
    bool read() override;
};

struct PwmChannel : Pwm
{
    uint8_t duty{0};

    bool set_freq(uint32_t freq) override;
    bool set_duty_cycle(uint8_t duty_cycle) override;
};

struct EncoderCount : Encoder
{
    int ticks{0};
    int status{0};

    int get_ticks() const override;
    int get_status() const override;
};

/**
 * @brief UART whose transmit side is stdout and whose receive side is fed
 *        by the simulator
 */
struct Terminal : Uart
{
    std::deque<uint8_t> rx;

    bool read(uint8_t& byte) override;
    bool write(std::span<const uint8_t> data) override;
};

class Plant
{
public:
    /** @brief Starts stowed, with the switch and fault lines already valid */
    Plant();

    /** @brief Advance the mechanism and sensors by dt_s seconds */
    void step(float dt_s);

    /** @brief IMU reading for the current instant */
    void imu(Vec3& accel, Vec3& gyro) const;

    /** @brief Apply launch-level acceleration for the next duration_ms */
    void boost(uint32_t duration_ms);

    Pin direction;
    Pin drv_z;
    Pin sleep;
    Pin fault;
    Pin retracted_switch;
    Pin extended_switch;
    PwmChannel pwm;
    EncoderCount encoder;
    Terminal terminal;

    bool jammed{false};            ///< mechanism cannot move
    bool driver_fault{false};      ///< nFAULT held low
    bool encoder_reversed{false};  ///< encoder counts the wrong way
    bool moving_base{false};       ///< IMU never reads at rest
    bool dead_switch{false};       ///< extended switch never closes

private:
    float position_{0.0f};
    float velocity_{0.0f};
    float boost_s_{0.0f};
};

}  // namespace LBR::Sim
