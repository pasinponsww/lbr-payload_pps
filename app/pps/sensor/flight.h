#pragma once
/**
 * @file flight.h
 * @brief Launch and landing detection from IMU data
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "debounce.h"
#include "sensors.h"

namespace LBR
{

/**
 * @struct FlightConfig
 * @brief Thresholds for launch and landing detection
 */
struct FlightConfig
{
    float launch_accel{25.0f};       ///< m/s^2 that counts as boost
    uint32_t launch_ms{200};         ///< boost must last this long
    float rest_accel_tol{1.5f};      ///< m/s^2 around 1 g that counts as rest
    float rest_gyro{0.15f};          ///< rad/s below which the payload is still
    uint32_t landed_ms{5000};        ///< rest must last this long
    uint32_t max_flight_ms{600000};  ///< landed is assumed after this long
};

class Flight
{
public:
    /**
     * @brief Build a detector
     * @param config detection thresholds
     */
    explicit Flight(const FlightConfig& config);

    /**
     * @brief Restart detection, call when entering Standby or Flight
     * @param now_ms current time, start of the flight timeout
     */
    void arm(uint32_t now_ms);

    /**
     * @brief Check for launch
     * @param frame sensor frame of this tick
     * @return true once boost acceleration has been held for launch_ms
     * @note Never true while the IMU is invalid.
     */
    bool launched(const SensorFrame& frame);

    /**
     * @brief Check for landing
     * @param frame sensor frame of this tick
     * @return true once the payload has been at rest for landed_ms, or
     *         max_flight_ms after arm() whatever the IMU says
     * @note The timeout is the fallback for a dead IMU.
     */
    bool landed(const SensorFrame& frame);

private:
    static constexpr float kGravity = 9.81f;

    static float magnitude(const Vec3& v);

    FlightConfig config_;
    Dwell condition_{};
    uint32_t armed_ms_{0};
};

}  // namespace LBR
