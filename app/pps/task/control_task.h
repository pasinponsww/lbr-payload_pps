#pragma once
/**
 * @file control_task.h
 * @brief Control tick: sample the sensors, step the state machine
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>
#include "pps.h"
#include "sensor/sensor_manager.h"

namespace LBR
{

class ControlTask
{
public:
    /**
     * @brief Build the task
     * @param sensors source of the sensor frame, must outlive this object
     * @param pps state machine to step, must outlive this object
     */
    ControlTask(SensorManager& sensors, Pps& pps);

    /**
     * @brief Run one control tick
     * @param now_ms current time
     * @note The IMU is only read when the state machine asks for it.
     */
    void run(uint32_t now_ms);

private:
    SensorManager& sensors_;
    Pps& pps_;
};

}  // namespace LBR
