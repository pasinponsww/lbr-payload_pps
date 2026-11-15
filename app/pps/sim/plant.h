#pragma once
/**
 * @file plant.h
 * @brief Simulated PPS hardware for running the firmware logic on a PC
 * @note Models the DRV8245 pins, a first-order DC motor, the quadrature
 *       encoder, both limit switches and the IMU bus, with flags to inject
 *       faults.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include <deque>
#include "encoder.h"
#include "gpio.h"
#include "i2c.h"
#include "imu_math.h"
#include "pwm.h"
#include "uart.h"

namespace LBR::Sim
{

/**
 * @struct Pin
 * @brief GPIO that just stores its level
 */
struct Pin : Gpio
{
    bool level{false};

    bool toggle() override;
    bool set(const bool active) override;
    bool read() override;
};

/**
 * @struct PwmChannel
 * @brief PWM output that just stores its duty cycle
 */
struct PwmChannel : Pwm
{
    uint8_t duty{0};

    bool set_freq(uint32_t freq) override;
    bool set_duty_cycle(uint8_t duty_cycle) override;
};

/**
 * @struct EncoderCount
 * @brief Encoder whose count is written by the plant
 */
struct EncoderCount : Encoder
{
    int ticks{0};
    int status{0};

    int get_ticks() const override;
    int get_status() const override;
};

/**
 * @struct Terminal
 * @brief UART whose transmit side is stdout and whose receive side is fed
 *        by the simulator
 */
struct Terminal : Uart
{
    std::deque<uint8_t> rx;

    bool read(uint8_t& byte) override;
    bool write(std::span<const uint8_t> data) override;
};

/**
 * @struct ImuBus
 * @brief I2C bus with a BNO055 on it that reports the plant's motion
 * @note Only the burst read of the data registers is answered, which is
 *       all the firmware uses at run time.
 */
struct ImuBus : I2c
{
    Vec3 accel{};  ///< m/s^2
    Vec3 gyro{};   ///< rad/s

    bool mem_read(std::span<uint8_t> data, const uint8_t reg_addr,
                  uint8_t dev_addr) override;
    bool mem_read(std::span<uint8_t> data, const uint16_t reg_addr,
                  uint8_t dev_addr) override;
    bool mem_write(std::span<const uint8_t> data, const uint8_t reg_addr,
                   uint8_t dev_addr) override;
    bool mem_write(std::span<const uint8_t> data, const uint16_t reg_addr,
                   uint8_t dev_addr) override;
    bool read(std::span<uint8_t> data, uint8_t dev_addr) override;
    bool write(std::span<const uint8_t> data, uint8_t dev_addr) override;

private:
    static constexpr uint8_t kDataRegister = 0x08;
    static constexpr size_t kGyroOffset = 12;
    static constexpr float kAccelScale = 100.0f;
    static constexpr float kGyroScale = 16.0f;

    static void encode(std::span<uint8_t> out, const Vec3& v, float scale);
};

class Plant
{
public:
    /**
     * @brief Build the plant stowed, with switch and fault lines valid
     */
    Plant();

    /**
     * @brief Advance the mechanism and sensors
     * @param dt_s time step in seconds
     */
    void step(float dt_s);

    /**
     * @brief Apply launch-level acceleration
     * @param duration_ms how long the boost lasts
     */
    void boost(uint32_t duration_ms);

    Pin direction;
    Pin drv_z;
    Pin sleep;
    Pin fault;
    Pin retracted_switch;
    Pin extended_switch;
    Pin led_red;
    Pin led_green;
    Pin led_blue;
    PwmChannel pwm;
    EncoderCount encoder;
    Terminal terminal;
    ImuBus imu_bus;

    bool jammed{false};            ///< mechanism cannot move
    bool driver_fault{false};      ///< nFAULT held low
    bool encoder_reversed{false};  ///< encoder counts the wrong way
    bool moving_base{false};       ///< IMU never reads at rest
    bool dead_switch{false};       ///< extended switch never closes

private:
    static constexpr float kMaxSpeed = 3000.0f;
    static constexpr float kDriveTau = 0.05f;
    static constexpr float kCoastTau = 0.30f;
    static constexpr float kTravel = 931.0f;
    static constexpr float kGravity = 9.81f;
    static constexpr float kBoostAccel = 40.0f;

    float position_{0.0f};
    float velocity_{0.0f};
    float boost_s_{0.0f};
};

}  // namespace LBR::Sim
